// Falcon - Mouse emulation joystick with Force
// Novint Falcon -> Pico 2 W (CP2102 serial) -> Xbox 360 controller
//
// CHANGES THIS REVISION:
//  - dt now measured with QueryPerformanceCounter (GetTickCount is 1ms-quantized;
//    at ~1kHz every IIR filter was being fed a wrong dt every frame)
//  - UPDATE_RATE_MS = 0 -> NO Sleep at all; the dhd USB transactions pace the loop
//    to 1000 Hz naturally (per Force Dimension support). UPDATE_RATE_MS >= 1 ->
//    precise QPC-deadline pacing (coarse Sleep then yield-spin) instead of raw Sleep().
//  - Controller serial packets decimated to SERIAL_SEND_MS (default 2ms = 500Hz);
//    button changes still send immediately. Haptic loop no longer stalls on WriteFile.
//  - RUMBLE_DECAY converted from per-FRAME to per-MILLISECOND (dt-based) so feel is
//    identical at any loop rate. Default changed 0.60 -> 0.70 to match old feel at ~700Hz.
//  - FIXED: ApplyForces had local static FRICTION_VEL_MIN/MAX shadowing the globals;
//    the global FRICTION_VEL_MIN=0.0006 was never in effect (0.003 was). Shadows
//    removed; global default set to 0.003 so behavior is unchanged but now tunable.
//  - Keyboard polling throttled to every 25ms; serial-thread RAW print gated on debug.
//  - Config file: loads falcon_config.txt at startup (auto-writes a template with all
//    defaults if missing). '#' and '//' comments. F8 = live reload while running.

#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <dhdc.h>

#pragma comment(lib, "winmm.lib")


// ── Serial port ────────────────────────────────────────────────────────────
static char  SERIAL_PORT[64] = "\\\\.\\COM6";
static DWORD SERIAL_BAUD = 115200;
static double SERIAL_SEND_MS = 2.0;   // controller packet interval (ms). 2ms = 500Hz, plenty for the Pico/Xbox side; button changes always send immediately

// ── Packet constants ───────────────────────────────────────────────────────
static const BYTE PACKET_IN_MAGIC = 0xAA;
static const BYTE PACKET_OUT_MAGIC = 0xBB;

// ── Controller config ──────────────────────────────────────────────────────
static int  UPDATE_RATE_MS = 0;  // 0 = unpaced, dhd USB calls throttle the loop (~1000Hz); >=1 = precise QPC pacing to that period. MUST stay int.
static bool INVERT_X = false;
static bool INVERT_Y = false;

// ──Velocity settings ──────────────────────────────────────────────────────
static double VEL_DEADZONE = 0.0001;
static double VEL_VLOW_SENS = 22.0;    // sensitivity for very slow corrections
static double VEL_VLOW_CURVE = 0.80;   // closer to 1.0 = more linear, less boosted
static double VEL_LOW_SENS = 18.0;  // sensitivity at low speeds
static double VEL_LOW_CURVE = 0.88; // curve for slow zone (lower = more boost)
static double VEL_HIGH_SENS = 12.0; // sensitivity at high speeds
static double VEL_HIGH_CURVE = 0.95; // curve for fast zone (1.0 = linear)
static double VEL_BLEND_VLOW = 0.004;  // below this = pure vlow zone
static double VEL_BLEND_LOW = 0.025; // below this = low speed
static double VEL_BLEND_HIGH = 0.15; // above this = high speed
static double SOFT_RAMP = 0.004;  // m/s width of ramp zone
static double SHORT_VEL_NOISE = 0.0005;  // higher suppresses more low-speed noise
static double VEL_VLOW_POS_SCALE = 7.0;  // position error scale

// Velocity estimator internals (previously hardcoded inside AxisState)
static double VEL_FC_VLOW = 9.0;    // smoothing cutoff (Hz) in the vlow bracket — higher = less latency, more jitter
static double VEL_FC_LOW = 18.0;    // cutoff at VEL_BLEND_LOW (smoothstepped from VEL_FC_VLOW)
static double VEL_FC_HIGH = 30.0;   // cutoff at VEL_BLEND_HIGH
static double VEL_FC_MAX = 55.0;    // cutoff at VEL_FC_SPEED_CEIL (fast motion = nearly raw)
static double SLOWREF_HZ = 1.2;     // drift rate of the slowRef low-pass reference (PosError anchor)
static int    VEL_WINDOW_SAMPLES = 6;  // velocity averaging window length (2..8; history arrays are sized 8). Time span is samples/loop-rate, so 6 = ~6ms at 1kHz
static int    VEL_ZERO_FRAMES = 10;    // consecutive sub-deadzone frames before smoothVel hard-zeros (frame-count, so loop-rate dependent by design — it's a settle guard)
static double POSBLEND_MAX_MULT = 2.0; // posBlendMax = VEL_BLEND_LOW * this — velocity magnitude below which position-error assist blends in
static double POSBLEND_SLEW_HZ = 6.0;  // slew on the position-blend amount — a one-frame velMag dip can't snap it

double posBlendMax = VEL_BLEND_LOW * POSBLEND_MAX_MULT;   // derived — recomputed after config load
static double VEL_RELEASE_MULT = 35.0; // decel cutoff multiplier — higher = snappier stop, lower = floatier
static double VEL_VERTICAL_SCALE = 1.0; // up/down (Z axis) velocity-zone output multiplier; left/right (Y axis) is unaffected. Independent of PUSH_VERTICAL_SCALE

// Flick capture: bank fast motion when saturated
static double FLICK_VEL_ON = 0.01;   // m/s above which motion is treated as a flick and banked (≈ where the stick saturates)
static double FLICK_GAIN = 4.0;    // how much banked over-speed becomes extra deflection
static double FLICK_DECAY = 0.92;   // per-ms decay of the carry tail — lower = shorter tail
static double FLICK_CARRY_MAX = 0.9;    // clamp on carry deflection so big flicks don't over-throw

// Push zone variables
static double PUSH_ENTER_RAD = 0.56; // percentage of work radius where push zone kicks in
static double PUSH_EXIT_RAD = 0.56; // percentage of work radius where push zone stops acting
static double PUSH_SPEED_BASE = 0.44; // how fast cursor moves when entering push zone
static double PUSH_SPEED_MAX = 6.0; // maximum push speed at full tilt
static double PUSH_VERTICAL_SCALE = 0.5; // up/down (Z axis) push-speed multiplier; left/right (Y axis) is unaffected
static double HALF_RANGE_MAX = 0.06; // how large work radius is (in m) — config changes take effect on next F11 recalibrate

// Velocity zone stuff
static double VEL_FC_SPEED_CEIL = 0.06; // velocity ceiling for top smoothing bracket (m/s) — NOT the work area
static double CALIB_FILL = 0.95;        // fraction of measured reach that maps to full deflection
static double CALIB_MIN = 0.025;       // reject implausibly small sweeps (m)
static double CALIB_MAX = 0.120;       // reject implausibly large sweeps (m)

// Push zone re-entry damping
static double PUSH_DAMP_COEFF = 0.9; // damping strength on direction reversal (0=none, 1=strong)
static double PUSH_DAMP_DECAY = 3.0; // how quickly damping fades (higher = faster)

// ── Boundary detent: a ridge you push over right before the push zone ─────────
static double DETENT_WIDTH = 0.12; // fraction of work radius over which the ridge builds
static double DETENT_PEAK_N = 1.6;  // ridge height in N — resistance grows, then releases at threshold
static double EDGE_HYST = 0.02;        // fraction of work radius of SPATIAL hysteresis on the pop latch at the push border. Once popped through, the ridge stays released (and the pump won't fire) until r retreats this far back inside — stops mm-wobble on the border from strobing the wall on/off

// ── Exit pump: a brief inward kick right at the border when leaving the push zone ──
static double PUMP_PEAK_N = 1.3;  // kick strength (N) as you cross back out of push
static double PUMP_DECAY = 0.90;  // per-ms decay of the kick — lower = snappier/shorter pump

//  Feed forward variables ────────────────────────────────────────────────────────
static double FRICTION_CANCEL = .14;    // feed forward force in Newtons
static double FRICTION_VEL_MIN = 0.003;  // engage threshold — WAS shadowed to 0.003 inside ApplyForces; default now matches the value you've actually been feeling
static double FRICTION_VEL_FULL = 0.004;  // assist reaches full here (fixed window, not tied to MIN)
static double FRICTION_VEL_MAX = 0.08;   // faded back out by here

static double STICTION_BREAK_N = 0.09;   // constant breakaway force (N)
static double STICTION_VEL_ON = 0.0005; // motion floor (= your SHORT_VEL_NOISE)

// Viscous drag cancellation — the Falcon's drag grows with speed (back-EMF, bearing
// viscosity); Coulomb cancel is constant-magnitude and can't touch that component.
static double VISCOUS_CANCEL = 0.0;   // N per (m/s) along velocity. 0 = off. Start ~1.0, raise until fast sweeps feel free; too high = self-driving/instability against the wall
static double VISCOUS_MAX_N = 0.6;    // hard cap on the viscous assist contribution (safety against runaway at high speed)

// Stiction dither — tiny high-frequency circular force that keeps the mechanism in
// kinetic (never static) friction, killing the re-grip catch at reversals and the
// start of micro-corrections. Fades out with speed: only needed near-stopped.
static double DITHER_N = 0.0;         // amplitude (N). 0 = off. Try 0.05-0.12; if the crosshair picks up jitter, lower this (or nudge SHORT_VEL_NOISE up)
static double DITHER_HZ = 70.0;       // dither frequency — high enough to not read as motion, low enough for the motors to render
static double DITHER_VEL_FADE = 0.010; // m/s above which dither is fully faded out

// ── Button 2 brace / preload ─────────────────────────────────────────────────
static double BTN2_X_FORCE = 1.6;  // constant X force while button 2 held (N) — preloads mechanism for fine corrections
static double BTN2_X_SIGN = 1.0;  // +1 or -1 to flip push direction

// ── Braced aim mode ──────────────────────────────────────────────────────────
// A grip button becomes a sensitivity brace instead of a controller button:
// while braced, velocity-zone stick output and push-zone speed are scaled down
// for precision aiming. The button is CONSUMED — masked out of the packet sent
// to the Pico, so the game never sees it.
static int    BRACE_BUTTON = 4;      // grip button 1-4 (4 = mask bit 8); 0 = disabled (button passes through normally)
static bool   BRACE_TOGGLE = false;  // false = hold to brace (ADS-style); true = press toggles on/off
static double BRACE_VEL_SCALE = 0.5; // velocity-zone stick output multiplier while braced
static double BRACE_PUSH_SCALE = 0.5;// push-zone speed multiplier while braced

//  Spring box ─────────────────────────────────────────────────────────────────────
static double FORCE_SPRING_START = 0.5; // percentage of work radius where spring force starts
static double FORCE_MAX_RAD = 0.88; // percentage of work radius where max force is achieved
static double FORCE_MAX_N = 8; // maximum allowable Force (in N)
static double FORCE_DAMPING = 1.0; // cut down on springiness
static double FORCE_EXPONENT = 2.3; // how you ramp to max force. lower = force builds earlier and harder
static double DAMP_VEL_MIN = 0.010; // below this speed = no ENHANCED wall damping (base FORCE_DAMPING always active)
static double DAMP_VEL_MAX = 0.020; // above this speed = full enhanced damping (smoothstepped between)
static double FORCE_YZ_CAP_N = 7.8; // hard cap on combined Y/Z force magnitude (spring + lateral rumble)

//  Gravity Compensation ────────────────────────────────────────────────────────────
static double GRAVITY_COMP_SCALE = 1.65;  // 1.0 = normal, 1.2 = 20% more, 0.8 = 20% less

// ──Ambient Rumble settings ──────────────────────────────────────────────────────
static double AMBIENT_LARGE_SCALE = 5.5;  // random rumble force scale
static double AMBIENT_SMALL_SCALE = 5.5;  // random rumble force scale
static double RUMBLE_DECAY = 0.70;  // NOW PER-MILLISECOND (dt-based, loop-rate independent). 0.70/ms ≈ old 0.60/frame at ~700Hz
static double RUMBLE_FORCE_SCALE = 2.0; // overall scale factor of rumble force
static double AMBIENT_TAIL_QUIET = 0.05;        // ambient is held off through the whole recoil rumble tail; the tail is "over" (ambient allowed again) once incoming rumble decays below this — adapts to large shots' longer tails instead of a fixed time window
static DWORD  AMBIENT_DIR_CHANGE_MS = 140;      // ms between new ambient target directions
static double AMBIENT_DIR_SLEW_HZ = 5.0;        // how fast ambient direction eases toward its target
static double AMBIENT_ATTACK_HZ = 7.0;          // ambient magnitude ramp-up cutoff
static double AMBIENT_RELEASE_HZ = 3.0;         // ambient magnitude ramp-down cutoff (slower = softer fade)

// ──Recoil settings ──────────────────────────────────────────────────────
static double RUMBLE_LARGE_SCALE = 2.4;  // recoil force scale
static double RUMBLE_SMALL_SCALE = 2.4;  // recoil force scale
static DWORD  RECOIL_WINDOW_MS = 250; // ms after btn 1 release to still catch trigger recoil
static double RECOIL_SUSTAIN_THRESHOLD = 0.5;  // liveMag above this = sustaining
static double RECOIL_CURVE = 0.50; // Recoil compressor -  0.3 boosts small recoils; 1.0 = linear
static double RECOIL_DECAY = 0.45;
static double RECOIL_DECAY_MIN = 0.25;  // decay for weak shots (fast cutoff)
static double RECOIL_DECAY_MAX = 0.75;  // decay for strong shots (long sustain)
static double RECOIL_MAG_SCALE = 20.0;  // liveMag value that maps to DECAY_MAX
static double RECOIL_AIM_DAMP = 0.4;  // stick sensitivity multiplier during recoil (0=frozen, 1=no effect)
static double RECOIL_DAMP_DECAY = 8.0;  // how fast aim damp fades per second
static double RECOIL_PUSH_SCALE = 20.0;  // sustained push force multiplier
static double BLEED_Y_SCALE = 0.5;   // recoil velocity bleed: fraction of Y velocity gated at full recoil (settle suppression)
static double BLEED_Z_DOWN = 0.8;    // stronger bleed on downward Z motion — gravity is the main tail-drift cause
static double BLEED_Z_UP = 0.4;      // lighter bleed on upward Z motion

static double RECOIL_ATTACK_SEC = 0.0;
static DWORD MIN_EDGE_INTERVAL_MS = 35;   // longer than a single-shot envelope

static DWORD  RECOIL_DIR_CHANGE_MS = 40;  // how often direction randomizes (same as ambient)
static double RECOIL_X_SCALE = 20.0;      // backwards kick strength
static double RECOIL_RELEASE_HZ = 12.0;   // release shaping: low-pass cutoff for the recoil push on the way DOWN only. Higher = sharper release, lower = smoother/longer. (Up-edges pass instantly so the kick stays crisp.)
static double RECOIL_VERTICAL = 10.0;  // upward force as fraction of recoil (0=none, 1=equal to X)
//static double RECOIL_Z_RETURN_RATE = 0.2;  // how fast debt bleeds back (higher = snappier return)
static double EDGE_THRESHOLD = 2.0;  //how much of a rising edge is considered a new impulse

static double RECOIL_TOPUP_BLEND = 0.6;   // how much of new peak boosts current force
static double RECOIL_YZ_ONSET_N = 2;     // how many rapid shots before Y/Z texture starts
static double RECOIL_YZ_DECAY = 0.15;  // how fast Y/Z texture fades between shots
static double RECOIL_YZ_SCALE = 0.15;   // Y/Z texture strength
static double RECOIL_YZ_DIR_SLEW_HZ = 6.0;  // how fast lateral texture direction eases toward target while active (lower = smoother)
static double RECOIL_YZ_GATE_HI = 0.8;      // incoming-rumble level that TURNS ON the lateral texture (strong rumble / the kick only)
static double RECOIL_YZ_GATE_LO = 0.5;      // level that TURNS OFF (hysteresis) — texture cuts before the weak decay tail to prevent jagged release
static double RECOIL_YZ_SMOOTH_HZ = 18.0;   // de-jags the active texture (follows packet steps smoothly instead of nudging per packet)
static double RECOIL_YZ_CUT_HZ = 45.0;      // fast hard-cut once the gate fails
static DWORD  RUMBLE_FRESH_MS = 40;         // a rumble packet must have arrived within this window to count as live; then clean release

// ── Rumble AGC: auto-normalize per-game rumble level ─────────────────────────
// Games report wildly different rumble amplitudes. The AGC tracks a slow running
// average of incoming magnitude and gains packets so that average sits at
// AGC_TARGET:   out = AGC_TARGET * (in / avg)^AGC_DYNAMICS
// DYNAMICS=1 is a pure level shift — every ratio is preserved, so strong shots
// keep their full separation from weak tails, just referenced to a common level.
// Applied at ingestion (inL/inS), so ALL downstream thresholds (recoil sustain,
// YZ gates, ambient quiet) compare against normalized units across titles.
static bool   AGC_ENABLE = false;  // master switch — off = raw passthrough, identical to previous behavior
static double AGC_TARGET = 0.5;    // rawMag level the running average is pulled to
static double AGC_ADAPT_SEC = 10.0;// adaptation time constant (s) — how fast "average" follows the game
static double AGC_FLOOR = 0.05;    // packets below this rawMag don't teach the average (silence must not drag it down)
static double AGC_GAIN_MIN = 0.33; // gain clamp — a loud game is attenuated at most this far
static double AGC_GAIN_MAX = 3.0;  // gain clamp — a quiet game's noise floor can't be amplified into phantom recoil
static double AGC_DYNAMICS = 1.0;  // 1 = ratios preserved exactly; >1 expands kick/tail separation; <1 compresses
static double AGC_TOLERANCE = 1.5; // dead band ratio: averages within TARGET/x .. TARGET*x are left RAW (gain 1). Outside, correction pulls the level to the band EDGE (not the center) so it engages smoothly with no jump at the threshold. 1.0 = always correct (old behavior)
static volatile double g_agcAvg = 0.0;  // learned average (0 = unseeded; first qualifying packet seeds it)


// ── Config file ──────────────────────────────────────────────────────────────
// falcon_config.txt is loaded at startup (and on F8). If the file doesn't exist,
// a template populated with all current defaults is written next to the exe.
// Format:  NAME = value      ('#' and '//' start comments; blank lines ignored)
// A different config can be given as the first command-line argument, e.g.
//   FalconJoyForce.exe destiny.txt
// which enables per-title profiles; F8 reloads whichever file was loaded.
static char g_configPath[MAX_PATH] = "falcon_config.txt";

enum CfgType { CFG_DBL, CFG_INT, CFG_DWORD, CFG_BOOL, CFG_STR, CFG_SECTION };
struct CfgEntry { const char* name; CfgType type; void* ptr; size_t strCap; const char* comment; };

#define C_D(x, c) { #x, CFG_DBL,     (void*)&x, 0,         c }
#define C_I(x, c) { #x, CFG_INT,     (void*)&x, 0,         c }
#define C_U(x, c) { #x, CFG_DWORD,   (void*)&x, 0,         c }
#define C_B(x, c) { #x, CFG_BOOL,    (void*)&x, 0,         c }
#define C_S(x, c) { #x, CFG_STR,     (void*)&x, sizeof(x), c }
#define SEC(t)    { t,  CFG_SECTION, NULL,      0,         NULL }

static CfgEntry g_cfgTable[] = {
    SEC("Serial / loop"),
    C_S(SERIAL_PORT,        "COM port device path (\\\\.\\COMn) -- restart required"),
    C_U(SERIAL_BAUD,        "baud rate -- restart required"),
    C_D(SERIAL_SEND_MS,     "controller packet interval (ms); 2 = 500Hz; button changes always send immediately"),
    C_I(UPDATE_RATE_MS,     "0 = unpaced, USB throttles loop to ~1000Hz; >=1 = precise QPC pacing to that period (ms)"),
    C_B(INVERT_X,           "flip stick X output"),
    C_B(INVERT_Y,           "flip stick Y output"),

    SEC("Velocity zones (aim response)"),
    C_D(VEL_DEADZONE,       "m/s below which velocity produces no stick output"),
    C_D(VEL_VLOW_SENS,      "sensitivity for very slow corrections"),
    C_D(VEL_VLOW_CURVE,     "closer to 1.0 = more linear, less boosted"),
    C_D(VEL_LOW_SENS,       "sensitivity at low speeds"),
    C_D(VEL_LOW_CURVE,      "curve for slow zone (lower = more boost)"),
    C_D(VEL_HIGH_SENS,      "sensitivity at high speeds"),
    C_D(VEL_HIGH_CURVE,     "curve for fast zone (1.0 = linear)"),
    C_D(VEL_BLEND_VLOW,     "m/s -- below this = pure vlow zone"),
    C_D(VEL_BLEND_LOW,      "m/s -- below this = low speed zone"),
    C_D(VEL_BLEND_HIGH,     "m/s -- above this = high speed zone"),
    C_D(SOFT_RAMP,          "m/s width of the ramp out of the deadzone"),
    C_D(SHORT_VEL_NOISE,    "higher suppresses more low-speed noise"),
    C_D(VEL_VLOW_POS_SCALE, "position-error assist scale at ultra-low speed"),
    C_D(VEL_RELEASE_MULT,   "decel cutoff multiplier -- higher = snappier stop, lower = floatier"),
    C_D(VEL_VERTICAL_SCALE, "Z-axis (up/down) velocity-zone output multiplier; Y unaffected"),

    SEC("Velocity estimator internals"),
    C_D(VEL_FC_VLOW,        "smoothing cutoff Hz in vlow bracket -- higher = less latency, more jitter"),
    C_D(VEL_FC_LOW,         "cutoff Hz at VEL_BLEND_LOW (smoothstepped from VEL_FC_VLOW)"),
    C_D(VEL_FC_HIGH,        "cutoff Hz at VEL_BLEND_HIGH"),
    C_D(VEL_FC_MAX,         "cutoff Hz at VEL_FC_SPEED_CEIL (fast motion = nearly raw)"),
    C_D(SLOWREF_HZ,         "drift rate of the slowRef low-pass (PosError anchor)"),
    C_I(VEL_WINDOW_SAMPLES, "velocity averaging window, 2..8 samples (= that many ms at 1kHz)"),
    C_I(VEL_ZERO_FRAMES,    "consecutive sub-deadzone frames before smoothVel hard-zeros"),
    C_D(POSBLEND_MAX_MULT,  "posBlendMax = VEL_BLEND_LOW * this -- speed below which position assist blends in"),
    C_D(POSBLEND_SLEW_HZ,   "slew on the position-blend amount -- one-frame dips can't snap it"),

    SEC("Flick capture"),
    C_D(FLICK_VEL_ON,       "m/s above which motion is banked as a flick (~ where the stick saturates)"),
    C_D(FLICK_GAIN,         "how much banked over-speed becomes extra deflection"),
    C_D(FLICK_DECAY,        "PER-MS decay of the carry tail -- lower = shorter tail"),
    C_D(FLICK_CARRY_MAX,    "clamp on carry deflection so big flicks don't over-throw"),

    SEC("Push zone"),
    C_D(PUSH_ENTER_RAD,     "fraction of work radius where push zone kicks in"),
    C_D(PUSH_EXIT_RAD,      "fraction of work radius where push zone stops acting"),
    C_D(PUSH_SPEED_BASE,    "cursor speed on entering push zone"),
    C_D(PUSH_SPEED_MAX,     "maximum push speed at full tilt"),
    C_D(PUSH_VERTICAL_SCALE,"Z-axis push-speed multiplier; Y unaffected"),
    C_D(HALF_RANGE_MAX,     "work radius (m) -- takes effect on next F11 recalibrate"),
    C_D(VEL_FC_SPEED_CEIL,  "velocity ceiling (m/s) for the top smoothing bracket -- NOT the work area"),
    C_D(CALIB_FILL,         "fraction of measured reach mapping to full deflection"),
    C_D(CALIB_MIN,          "reject implausibly small calibration sweeps (m)"),
    C_D(CALIB_MAX,          "reject implausibly large calibration sweeps (m)"),
    C_D(PUSH_DAMP_COEFF,    "stick damping strength on push-zone direction reversal (0=none, 1=strong)"),
    C_D(PUSH_DAMP_DECAY,    "how quickly reversal damping fades (higher = faster)"),

    SEC("Boundary detent & exit pump"),
    C_D(DETENT_WIDTH,       "fraction of work radius over which the ridge builds"),
    C_D(DETENT_PEAK_N,      "ridge height (N) -- this IS the wall at the border (spring is ~0.1N there); raise for a firmer stop"),
    C_D(EDGE_HYST,          "spatial hysteresis (fraction of work radius) on the border pop latch; stops edge wobble strobing the ridge/pump"),
    C_D(PUMP_PEAK_N,        "inward kick strength (N) crossing back out of push"),
    C_D(PUMP_DECAY,         "PER-MS decay of the kick -- lower = snappier/shorter pump"),

    SEC("Friction / stiction compensation"),
    C_D(FRICTION_CANCEL,    "Coulomb feed-forward force (N) along motion"),
    C_D(FRICTION_VEL_MIN,   "m/s where assist engages"),
    C_D(FRICTION_VEL_FULL,  "m/s where assist reaches full (fixed window, not tied to MIN)"),
    C_D(FRICTION_VEL_MAX,   "m/s by which assist has faded back out"),
    C_D(STICTION_BREAK_N,   "constant breakaway force (N)"),
    C_D(STICTION_VEL_ON,    "motion floor for breakaway (= SHORT_VEL_NOISE)"),
    C_D(VISCOUS_CANCEL,     "N per m/s -- cancels speed-proportional drag. 0=off; start ~1.0; too high = self-driving"),
    C_D(VISCOUS_MAX_N,      "hard cap on viscous assist (safety against runaway)"),
    C_D(DITHER_N,           "stiction dither amplitude (N). 0=off; try 0.05-0.12; lower if crosshair jitters"),
    C_D(DITHER_HZ,          "dither frequency -- high enough not to read as motion"),
    C_D(DITHER_VEL_FADE,    "m/s above which dither is fully faded out"),

    SEC("Button 2 brace / preload"),
    C_D(BTN2_X_FORCE,       "constant X force while button 2 held (N) -- preloads mechanism for fine corrections"),
    C_D(BTN2_X_SIGN,        "+1 or -1 to flip preload direction"),

    SEC("Braced aim mode"),
    C_I(BRACE_BUTTON,       "grip button 1-4 that enters braced aim (4 = mask bit 8); 0 = disabled. Button is consumed, not sent to the game"),
    C_B(BRACE_TOGGLE,       "false = hold to brace (ADS-style); true = press toggles on/off"),
    C_D(BRACE_VEL_SCALE,    "velocity-zone stick output multiplier while braced"),
    C_D(BRACE_PUSH_SCALE,   "push-zone speed multiplier while braced"),

    SEC("Spring wall"),
    C_D(FORCE_SPRING_START, "fraction of work radius where spring force starts"),
    C_D(FORCE_MAX_RAD,      "fraction of work radius where max force is reached"),
    C_D(FORCE_MAX_N,        "maximum wall force (N)"),
    C_D(FORCE_DAMPING,      "bidirectional radial damping -- removes wall-contact buzz energy; higher = deader wall"),
    C_D(FORCE_EXPONENT,     "force ramp shape -- lower = builds earlier and harder"),
    C_D(DAMP_VEL_MIN,       "m/s below which no ENHANCED damping (base FORCE_DAMPING always active)"),
    C_D(DAMP_VEL_MAX,       "m/s above which full enhanced damping (smoothstepped)"),
    C_D(FORCE_YZ_CAP_N,     "hard cap on combined Y/Z force (spring + lateral rumble)"),

    SEC("Gravity compensation"),
    C_D(GRAVITY_COMP_SCALE, "1.0 = physical; >1 preloads joints upward and adds asymmetric drag"),

    SEC("Ambient rumble"),
    C_D(AMBIENT_LARGE_SCALE,  "large-motor ambient force scale"),
    C_D(AMBIENT_SMALL_SCALE,  "small-motor ambient force scale"),
    C_D(RUMBLE_DECAY,         "PER-MS retention of incoming rumble (0.70/ms ~= old 0.60/frame at 700Hz)"),
    C_D(RUMBLE_FORCE_SCALE,   "overall scale factor of rumble force"),
    C_D(AMBIENT_TAIL_QUIET,   "ambient held off until incoming rumble decays below this (adapts to shot size)"),
    C_U(AMBIENT_DIR_CHANGE_MS,"ms between new ambient target directions"),
    C_D(AMBIENT_DIR_SLEW_HZ,  "how fast ambient direction eases toward its target"),
    C_D(AMBIENT_ATTACK_HZ,    "ambient magnitude ramp-up cutoff"),
    C_D(AMBIENT_RELEASE_HZ,   "ambient magnitude ramp-down cutoff (slower = softer fade)"),

    SEC("Recoil"),
    C_D(RUMBLE_LARGE_SCALE,   "recoil scale for large-motor rumble"),
    C_D(RUMBLE_SMALL_SCALE,   "recoil scale for small-motor rumble"),
    C_U(RECOIL_WINDOW_MS,     "ms after btn1 release to still catch trigger recoil -- shorter = snappier, <100 may miss single shots"),
    C_D(RECOIL_SUSTAIN_THRESHOLD, "liveMag above this = still firing; PRIMARY per-title linger knob -- raise so weak tail packets can't sustain"),
    C_D(RECOIL_CURVE,         "recoil compressor -- 0.3 boosts small recoils; 1.0 = linear"),
    C_D(RECOIL_DECAY,         "initial per-ms decay (overwritten per shot by MIN..MAX mapping)"),
    C_D(RECOIL_DECAY_MIN,     "per-ms decay for weak shots (fast cutoff)"),
    C_D(RECOIL_DECAY_MAX,     "per-ms decay for strong shots (long sustain)"),
    C_D(RECOIL_MAG_SCALE,     "liveMag that maps to DECAY_MAX -- raise so fewer shots get the long sustain"),
    C_D(RECOIL_AIM_DAMP,      "stick sensitivity multiplier during recoil (0=frozen, 1=no effect)"),
    C_D(RECOIL_DAMP_DECAY,    "how fast aim damp fades per second -- raise if aim feels muddy after bursts"),
    C_D(RECOIL_PUSH_SCALE,    "sustained push force multiplier"),
    C_D(BLEED_Y_SCALE,        "velocity bleed: fraction of Y velocity gated at full recoil (settle suppression)"),
    C_D(BLEED_Z_DOWN,         "stronger bleed on downward Z -- gravity is the main tail-drift cause"),
    C_D(BLEED_Z_UP,           "lighter bleed on upward Z"),
    C_D(RECOIL_ATTACK_SEC,    "attack envelope length (0 = instant kick)"),
    C_U(MIN_EDGE_INTERVAL_MS, "minimum ms between edges counted as new impulses"),
    C_U(RECOIL_DIR_CHANGE_MS, "ms between lateral texture direction randomizations"),
    C_D(RECOIL_X_SCALE,       "backwards kick strength"),
    C_D(RECOIL_RELEASE_HZ,    "down-ramp low-pass; higher = sharper release (past ~30 the sawtooth buzz returns); up-edges pass instantly"),
    C_D(RECOIL_VERTICAL,      "upward force as fraction of recoil (0=none)"),
    C_D(EDGE_THRESHOLD,       "rising-edge size considered a new impulse"),
    C_D(RECOIL_TOPUP_BLEND,   "how much of a new peak boosts current force"),
    C_D(RECOIL_YZ_ONSET_N,    "rapid shots before Y/Z texture starts"),
    C_D(RECOIL_YZ_DECAY,      "how fast Y/Z texture fades between shots"),
    C_D(RECOIL_YZ_SCALE,      "Y/Z texture strength"),
    C_D(RECOIL_YZ_DIR_SLEW_HZ,"lateral texture direction slew while active (lower = smoother)"),
    C_D(RECOIL_YZ_GATE_HI,    "incoming level that turns lateral texture ON (the strong kick only)"),
    C_D(RECOIL_YZ_GATE_LO,    "level that turns it OFF (hysteresis) -- cuts before the weak tail to prevent jagged release"),
    C_D(RECOIL_YZ_SMOOTH_HZ,  "de-jags the active texture (follows packet steps smoothly)"),
    C_D(RECOIL_YZ_CUT_HZ,     "fast hard-cut once the gate fails"),
    C_U(RUMBLE_FRESH_MS,      "rumble packet must arrive within this window to count as live; then clean release"),

    SEC("Rumble AGC (auto per-game normalization)"),
    C_B(AGC_ENABLE,           "normalize each game's rumble level to AGC_TARGET; off = raw passthrough"),
    C_D(AGC_TARGET,           "rawMag level the running average is pulled to -- all recoil/ambient thresholds then live in these units"),
    C_D(AGC_ADAPT_SEC,        "adaptation time constant (s); first shot seeds instantly, then average follows at this rate"),
    C_D(AGC_FLOOR,            "packets below this don't teach the average (silence must not drag it down)"),
    C_D(AGC_GAIN_MIN,         "gain clamp -- loud games attenuated at most this far"),
    C_D(AGC_GAIN_MAX,         "gain clamp -- quiet games amplified at most this far (keeps noise floor from becoming phantom recoil)"),
    C_D(AGC_DYNAMICS,         "1 = ratios preserved exactly; >1 expands kick/tail separation; <1 compresses"),
    C_D(AGC_TOLERANCE,        "dead band: averages within TARGET/x..TARGET*x stay raw; outside, level pulled to the band edge. 1.0 = always correct"),
};
static const int g_cfgCount = (int)(sizeof(g_cfgTable) / sizeof(g_cfgTable[0]));

static void CfgTrim(char* s) {
    char* p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1])) s[--n] = 0;
}

static void CfgWriteEntry(FILE* f, const CfgEntry& e) {
    if (e.type == CFG_SECTION) {
        fprintf(f, "\n# ---------- %s ----------\n", e.name);
        return;
    }
    char val[128];
    switch (e.type) {
    case CFG_DBL:   snprintf(val, sizeof(val), "%g", *(double*)e.ptr); break;
    case CFG_INT:   snprintf(val, sizeof(val), "%d", *(int*)e.ptr); break;
    case CFG_DWORD: snprintf(val, sizeof(val), "%lu", (unsigned long)(*(DWORD*)e.ptr)); break;
    case CFG_BOOL:  snprintf(val, sizeof(val), "%s", *(bool*)e.ptr ? "true" : "false"); break;
    case CFG_STR:   snprintf(val, sizeof(val), "%s", (char*)e.ptr); break;
    default:        val[0] = 0; break;
    }
    if (e.comment && *e.comment)
        fprintf(f, "%-26s = %-10s # %s\n", e.name, val, e.comment);
    else
        fprintf(f, "%-26s = %s\n", e.name, val);
}

static void WriteConfigTemplate(const char* path) {
    FILE* f = fopen(path, "w");
    if (!f) { printf("Config: could not write template %s\n", path); return; }
    fprintf(f, "# FalconJoyForce configuration\n");
    fprintf(f, "# Edit values and restart, or press F8 in the running program to reload live.\n");
    fprintf(f, "# Anything after '#' or '//' is a comment. Missing keys keep their compiled defaults.\n");
    for (int i = 0; i < g_cfgCount; i++) CfgWriteEntry(f, g_cfgTable[i]);
    fclose(f);
    printf("Config: wrote template with defaults -> %s\n", path);
}

static bool LoadConfig(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) {
        printf("Config: %s not found, using compiled defaults.\n", path);
        WriteConfigTemplate(path);
        return false;
    }
    char line[512];
    int applied = 0;
    bool present[sizeof(g_cfgTable) / sizeof(g_cfgTable[0])] = {};
    while (fgets(line, sizeof(line), f)) {
        char* c = strchr(line, '#');   if (c) *c = 0;   // strip comments
        c = strstr(line, "//");        if (c) *c = 0;
        char* eq = strchr(line, '=');  if (!eq) continue;
        *eq = 0;
        char* key = line;
        char* val = eq + 1;
        CfgTrim(key); CfgTrim(val);
        if (!*key || !*val) continue;

        bool found = false;
        for (int i = 0; i < g_cfgCount; i++) {
            CfgEntry& e = g_cfgTable[i];
            if (e.type == CFG_SECTION) continue;
            if (_stricmp(e.name, key) != 0) continue;
            found = true;
            switch (e.type) {
            case CFG_DBL:   *(double*)e.ptr = atof(val); break;
            case CFG_INT:   *(int*)e.ptr = atoi(val); break;
            case CFG_DWORD: *(DWORD*)e.ptr = (DWORD)strtoul(val, NULL, 10); break;
            case CFG_BOOL:  *(bool*)e.ptr = (_stricmp(val, "true") == 0 || _stricmp(val, "1") == 0
                || _stricmp(val, "yes") == 0 || _stricmp(val, "on") == 0); break;
            case CFG_STR:   strncpy((char*)e.ptr, val, e.strCap - 1);
                ((char*)e.ptr)[e.strCap - 1] = 0; break;
            }
            applied++;
            present[i] = true;
            break;
        }
        if (!found) printf("Config: unknown key '%s' (ignored)\n", key);
    }
    fclose(f);
    printf("Config: %d value(s) loaded from %s\n", applied, path);
    int missing = 0;
    for (int i = 0; i < g_cfgCount; i++)
        if (g_cfgTable[i].type != CFG_SECTION && !present[i]) missing++;
    if (missing > 0) {
        // Self-healing config: new tunables added since this file was created are
        // appended (with their comments and current defaults) under a marker, so
        // the file stays complete across code updates without touching tuned values.
        FILE* fa = fopen(path, "a");
        if (fa) {
            fprintf(fa, "\n# ---------- Added by program: new keys since this file was created ----------\n");
            for (int i = 0; i < g_cfgCount; i++) {
                if (g_cfgTable[i].type == CFG_SECTION || present[i]) continue;
                CfgWriteEntry(fa, g_cfgTable[i]);
            }
            fclose(fa);
            printf("Config: appended %d new key(s) with defaults to %s:\n  ", missing, path);
        }
        else {
            printf("Config: %d key(s) missing and file not writable (defaults in effect):\n  ", missing);
        }
        int shown = 0;
        for (int i = 0; i < g_cfgCount; i++) {
            if (g_cfgTable[i].type == CFG_SECTION || present[i]) continue;
            printf("%s%s", shown++ ? ", " : "", g_cfgTable[i].name);
        }
        printf("\n");
    }
    return true;
}

// Anything derived from config values goes here — called after every (re)load.
static void ApplyDerivedConfig() {
    posBlendMax = VEL_BLEND_LOW * POSBLEND_MAX_MULT;
    if (VEL_WINDOW_SAMPLES < 2) VEL_WINDOW_SAMPLES = 2;   // history arrays are sized 8 —
    if (VEL_WINDOW_SAMPLES > 8) VEL_WINDOW_SAMPLES = 8;   // clamp so indexing can never overrun
    if (VEL_ZERO_FRAMES < 1) VEL_ZERO_FRAMES = 1;
}


// ── Recoil impulse queue ───────────────────────────────────────────────────
#define RECOIL_QUEUE_SIZE 64
static double g_recoilQueue[RECOIL_QUEUE_SIZE] = {};
static int    g_recoilQHead = 0;
static int    g_recoilQTail = 0;
static DWORD  g_lastRumbleTime = 0;
static CRITICAL_SECTION g_recoilCS;

static void RecoilEnqueue(double peak) {
    EnterCriticalSection(&g_recoilCS);
    int next = (g_recoilQHead + 1) % RECOIL_QUEUE_SIZE;
    if (next != g_recoilQTail) {
        g_recoilQueue[g_recoilQHead] = peak;
        g_recoilQHead = next;
    }
    LeaveCriticalSection(&g_recoilCS);
}

static bool RecoilDequeue(double& peak) {
    EnterCriticalSection(&g_recoilCS);
    if (g_recoilQTail == g_recoilQHead) {
        LeaveCriticalSection(&g_recoilCS);
        return false;
    }
    peak = g_recoilQueue[g_recoilQTail];
    g_recoilQTail = (g_recoilQTail + 1) % RECOIL_QUEUE_SIZE;
    LeaveCriticalSection(&g_recoilCS);
    return true;
}

// ── Runtime state ──────────────────────────────────────────────────────────
static volatile float g_rumbleLarge = 0.0f;
static volatile float g_rumbleSmall = 0.0f;
static double g_recoilForce = 0.0;
static DWORD  g_btn1Released = 0;
static double g_pushDamp = 0.0;
static volatile float g_rumbleLargePeak = 0.0f;  // undecayed, for sustain check
static volatile float g_rumbleSmallPeak = 0.0f;
static double g_recoilPeakOrig = 0.0;  // peak at start of impulse, never decayed
static double g_recoilDampEnv = 0.0;
static double g_recoilPeak = 0.0;
static double g_recoilAttack = 0.0;
static bool   g_recoilFiring = false;
static double g_recoilYZForce = 0.0;  // current Y/Z texture magnitude
static int    g_rapidShotCount = 0;    // consecutive shots that interrupted an impulse
static double g_recoilYZDirY = 1.0;
static double g_recoilYZDirZ = 0.0;
static double g_recoilYZTgtY = 1.0;   // target lateral direction (actual dir slews toward it)
static double g_recoilYZTgtZ = 0.0;
static double g_recoilYZLevel = 0.0;  // smoothed, hysteresis-gated lateral texture magnitude
static bool   g_recoilTailActive = false;  // true while a recoil's rumble tail is still arriving (ambient held off)
static DWORD  g_lastYZDirChange = 0;
static double          g_recoilDecay = 0.55;
static volatile bool   g_running = true;
static volatile bool   g_debugMode = false;   // shared with serial thread so RAW prints don't fight the status line
static volatile uint8_t g_btnMask = 0;
static volatile uint8_t g_dhdButtons = 0;
//static volatile DWORD  g_lastRumbleTime = 0;

// ── Serial ─────────────────────────────────────────────────────────────────
static HANDLE g_hSerial = INVALID_HANDLE_VALUE;

static bool SerialOpen(const char* port, DWORD baud) {
    g_hSerial = CreateFileA(port, GENERIC_READ | GENERIC_WRITE,
        0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (g_hSerial == INVALID_HANDLE_VALUE) return false;
    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(g_hSerial, &dcb);
    dcb.BaudRate = baud; dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT; dcb.Parity = NOPARITY;
    SetCommState(g_hSerial, &dcb);
    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = 1;
    timeouts.ReadTotalTimeoutConstant = 2;
    timeouts.WriteTotalTimeoutConstant = 10;
    SetCommTimeouts(g_hSerial, &timeouts);
    return true;
}

static void SerialClose() {
    if (g_hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(g_hSerial);
        g_hSerial = INVALID_HANDLE_VALUE;
    }
}

static void SerialWrite(const BYTE* data, int len) {
    if (g_hSerial == INVALID_HANDLE_VALUE) return;
    DWORD written = 0;
    WriteFile(g_hSerial, data, len, &written, NULL);
}

static DWORD WINAPI SerialReaderThread(LPVOID) {
    BYTE buf[4]; int pos = 0;
    while (g_hSerial != INVALID_HANDLE_VALUE) {
        BYTE b; DWORD n = 0;
        if (!ReadFile(g_hSerial, &b, 1, &n, NULL) || n == 0) { Sleep(1); continue; }
        if (pos == 0 && b != PACKET_OUT_MAGIC) continue;
        buf[pos++] = b;
        if (pos == 4) {
            pos = 0;
            if ((buf[1] ^ buf[2]) == buf[3]) {
                float inL = buf[1] / 255.0f;
                float inS = buf[2] / 255.0f;
                double rawMag = (inL * 1.0 + inS * 0.5);
                double preMag = rawMag;   // pre-AGC, for the debug readout

                // ── Rumble AGC ────────────────────────────────────────────────
                // Teach the running average (loud-enough packets only, time-based
                // rate), then gain this packet so the game's average level lands
                // on AGC_TARGET. Both motors share one gain so their ratio holds.
                if (AGC_ENABLE && rawMag > 1e-6) {
                    if (rawMag > AGC_FLOOR) {
                        static DWORD lastTeach = 0;
                        DWORD nowT = GetTickCount();
                        double dtp = (lastTeach == 0) ? 0.0 : (nowT - lastTeach) / 1000.0;
                        if (dtp > 0.5) dtp = 0.5;   // long silences don't cause a giant step
                        lastTeach = nowT;
                        if (g_agcAvg <= 0.0) g_agcAvg = rawMag;   // first shot seeds instantly
                        else {
                            double a = 1.0 - exp(-dtp / AGC_ADAPT_SEC);
                            g_agcAvg += a * (rawMag - g_agcAvg);
                        }
                    }
                    if (g_agcAvg > 1e-6) {
                        // Dead band: inside TARGET/TOL..TARGET*TOL the effective
                        // target IS the learned average → gain 1, raw passthrough.
                        // Outside, effTarget clamps to the band edge, so correction
                        // only removes the excess beyond "reasonable" and shrinks
                        // smoothly to zero as a game approaches the band.
                        double tol = (AGC_TOLERANCE < 1.0) ? 1.0 : AGC_TOLERANCE;
                        double lo = AGC_TARGET / tol;
                        double hi = AGC_TARGET * tol;
                        double effTarget = g_agcAvg < lo ? lo : (g_agcAvg > hi ? hi : g_agcAvg);
                        double norm = effTarget * pow(rawMag / g_agcAvg, AGC_DYNAMICS);
                        double gain = norm / rawMag;
                        if (gain < AGC_GAIN_MIN) gain = AGC_GAIN_MIN;
                        if (gain > AGC_GAIN_MAX) gain = AGC_GAIN_MAX;
                        inL = (float)(inL * gain);
                        inS = (float)(inS * gain);
                        rawMag *= gain;
                    }
                }

                if (inL > g_rumbleLarge) g_rumbleLarge = inL;
                if (inS > g_rumbleSmall) g_rumbleSmall = inS;
                g_rumbleLargePeak = inL;   // ← raw peak, no decay
                g_rumbleSmallPeak = inS;
                if (g_debugMode) {   // console I/O from this thread contends the main status line — debug only
                    static DWORD lastRawPrint = 0;
                    if (GetTickCount() - lastRawPrint > 100) {
                        lastRawPrint = GetTickCount();
                        printf("\nRAW: L=%.3f S=%.3f mag=%.3f (pre-AGC=%.3f avg=%.3f)\n",
                            inL, inS, rawMag, preMag, g_agcAvg);
                    }
                }
                double newRecoil = pow(rawMag, RECOIL_CURVE) * RUMBLE_FORCE_SCALE * FORCE_MAX_N;
                RecoilEnqueue(newRecoil);
                g_lastRumbleTime = GetTickCount();
            }
        }
    }
    return 0;
}

static void SendControllerPacket(int16_t lx, int16_t ly, uint8_t buttons) {
    BYTE pkt[7];
    pkt[0] = PACKET_IN_MAGIC;
    pkt[1] = (BYTE)((lx >> 8) & 0xFF);
    pkt[2] = (BYTE)(lx & 0xFF);
    pkt[3] = (BYTE)((ly >> 8) & 0xFF);
    pkt[4] = (BYTE)(ly & 0xFF);
    pkt[5] = buttons;
    pkt[6] = pkt[1] ^ pkt[2] ^ pkt[3] ^ pkt[4] ^ pkt[5];
    SerialWrite(pkt, 7);
}

// ── Per-axis state ─────────────────────────────────────────────────────────
struct AxisState {
    double absMin = 999.0;
    double absMax = -999.0;
    bool   seeded = false;
    double estCenter = 0.0;
    double halfRange = HALF_RANGE_MAX;
    double lastPos = 0.0;
    double smoothVel = 0.0;
    double rawVel = 0.0;
    double flickCarry = 0.0;   // banked fast-motion deflection, paid out as a tail
    int    zeroCount = 0;
    double slowRef = 0.0;
    bool   slowRefSeeded = false;

    // window length is the global VEL_WINDOW_SAMPLES (config, 2..8); arrays fixed at 8
    double posHistory[8] = {};
    double timeHistory[8] = {};
    int    posIdx = 0;
    bool   posSeeded = false;

    void UpdateReach(double pos) {
        if (pos < absMin) absMin = pos;
        if (pos > absMax) absMax = pos;
        if (absMax > absMin) { seeded = true; estCenter = (absMin + absMax) / 2.0; }
    }

    void UpdateSlowRef(double pos, double dt) {
        if (!slowRefSeeded) { slowRef = pos; slowRefSeeded = true; return; }
        double alpha = 1.0 - exp(-2.0 * 3.14159265 * SLOWREF_HZ * dt);
        slowRef += alpha * (pos - slowRef);
    }

    double PosError(double pos) const {
        if (halfRange < 0.001) return 0.0;
        double e = (pos - slowRef) / halfRange;
        return e < -1.0 ? -1.0 : (e > 1.0 ? 1.0 : e);
    }

    void UpdateVelocity(double pos, double dt) {
        if (!posSeeded) {
            for (int i = 0; i < VEL_WINDOW_SAMPLES; i++) {
                posHistory[i] = pos;
                timeHistory[i] = dt > 0.0 ? dt : 0.001;
            }
            posSeeded = true;
        }

        posHistory[posIdx] = pos;
        timeHistory[posIdx] = dt > 0.0 ? dt : 0.001;
        posIdx = (posIdx + 1) % VEL_WINDOW_SAMPLES;

        double oldPos = posHistory[posIdx];
        double elapsed = 0.0;
        for (int i = 0; i < VEL_WINDOW_SAMPLES; i++) elapsed += timeHistory[i];
        double avgVel = (elapsed > 0.0001) ? (pos - oldPos) / elapsed : 0.0;

        int prev2 = (posIdx + VEL_WINDOW_SAMPLES - 2) % VEL_WINDOW_SAMPLES;
        double t2 = timeHistory[(posIdx + VEL_WINDOW_SAMPLES - 1) % VEL_WINDOW_SAMPLES]
            + timeHistory[(posIdx + VEL_WINDOW_SAMPLES - 2) % VEL_WINDOW_SAMPLES];
        double shortVel = (t2 > 0.0001) ? (pos - posHistory[prev2]) / t2 : 0.0;

        if (fabs(shortVel) > SHORT_VEL_NOISE && fabs(shortVel) > fabs(avgVel))
            rawVel = shortVel;
        else
            rawVel = avgVel;

        //Alpha section - smoothing at each speed bracket (cutoffs now configurable)
        double velSpeed = fabs(rawVel);
        double fc;
        if (velSpeed < VEL_BLEND_VLOW) {
            fc = VEL_FC_VLOW;
        }
        else if (velSpeed < VEL_BLEND_LOW) {
            double t = (velSpeed - VEL_BLEND_VLOW) / (VEL_BLEND_LOW - VEL_BLEND_VLOW);
            t = t * t * (3.0 - 2.0 * t);
            fc = VEL_FC_VLOW + (VEL_FC_LOW - VEL_FC_VLOW) * t;
        }
        else if (velSpeed < VEL_BLEND_HIGH) {
            double t = (velSpeed - VEL_BLEND_LOW) / (VEL_BLEND_HIGH - VEL_BLEND_LOW);
            t = t * t * (3.0 - 2.0 * t);
            fc = VEL_FC_LOW + (VEL_FC_HIGH - VEL_FC_LOW) * t;
        }
        else {
            double t = (velSpeed - VEL_BLEND_HIGH) / (VEL_FC_SPEED_CEIL - VEL_BLEND_HIGH);
            t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
            fc = VEL_FC_HIGH + (VEL_FC_MAX - VEL_FC_HIGH) * t;
        }
        double fcRel = fc;
        if (fabs(rawVel) < fabs(smoothVel)) fcRel = fc * VEL_RELEASE_MULT; // decelerating → collapse faster
        double alpha = 1.0 - exp(-2.0 * 3.14159265 * fcRel * dt);
        smoothVel += alpha * (rawVel - smoothVel);

        if (fabs(rawVel) < VEL_DEADZONE && fabs(smoothVel) < VEL_DEADZONE * 2.0) {
            zeroCount++;
            if (zeroCount > VEL_ZERO_FRAMES) smoothVel = 0.0;
        }
        else {
            zeroCount = 0;
        }

        lastPos = pos;
    }

    // ── Flick capture ────────────────────────────────────────────────────────
    // When the hand moves faster than the stick can express (saturation), bank the
    // over-speed and pay it back as a brief decaying carry, so a quick flick delivers
    // the rotation its displacement deserves. Below the threshold nothing banks, so
    // slow aiming and snappy stops are untouched. A reversal cancels pending carry.
    void UpdateFlick(double dt) {
        double sp = fabs(smoothVel);
        double excess = sp - FLICK_VEL_ON;
        if (excess < 0.0) excess = 0.0;
        double target = (smoothVel >= 0.0 ? 1.0 : -1.0) * excess * FLICK_GAIN;

        if (flickCarry != 0.0 && fabs(smoothVel) > VEL_DEADZONE) {
            double velSign = (smoothVel >= 0.0) ? 1.0 : -1.0;
            if (velSign * flickCarry < 0.0) flickCarry = 0.0;
        }

        if (fabs(target) > fabs(flickCarry)) flickCarry = target;
        else flickCarry *= pow(FLICK_DECAY, dt * 1000.0);
        if (flickCarry > FLICK_CARRY_MAX) flickCarry = FLICK_CARRY_MAX;
        if (flickCarry < -FLICK_CARRY_MAX) flickCarry = -FLICK_CARRY_MAX;
        if (fabs(flickCarry) < 0.001) flickCarry = 0.0;
    }

    double Offset(double pos) const {
        if (halfRange < 0.001) return 0.0;
        double o = (pos - estCenter) / halfRange;
        return o < -1.0 ? -1.0 : (o > 1.0 ? 1.0 : o);
    }
};

// ── Push zone ─────────────────────────────────────────────────────────────
struct PushState2D {
    bool inPushY = false;
    bool inPushZ = false;
    double entryVelY = 0.0;  // velocity captured at moment of entry
    double entryVelZ = 0.0;

    bool Update(double offY, double offZ, double velY, double velZ, double& outX, double& outY) {
        // Y axis
        if (inPushY) {
            if (fabs(offY) < PUSH_EXIT_RAD) { inPushY = false; entryVelY = 0.0; }
        }
        else {
            if (fabs(offY) >= PUSH_ENTER_RAD) {
                inPushY = true;
                entryVelY = outX;  // capture current stick output at entry
            }
        }

        // Z axis
        if (inPushZ) {
            if (fabs(offZ) < PUSH_EXIT_RAD) { inPushZ = false; entryVelZ = 0.0; }
        }
        else {
            if (fabs(offZ) >= PUSH_ENTER_RAD) {
                inPushZ = true;
                entryVelZ = outY;  // capture current stick output at entry
            }
        }

        bool inPush = inPushY || inPushZ;
        outX = outY = 0.0;

        if (inPushY) {
            double excess = (fabs(offY) - PUSH_ENTER_RAD) / (1.0 - PUSH_ENTER_RAD);
            excess = excess < 0.0 ? 0.0 : (excess > 1.0 ? 1.0 : excess);
            // start at entry velocity, scale up toward PUSH_SPEED_MAX with depth
            double sign = offY >= 0.0 ? 1.0 : -1.0;
            double base = fabs(entryVelY) > 0.01 ? fabs(entryVelY) : PUSH_SPEED_BASE;
            double mag = base + excess * (PUSH_SPEED_MAX - base);
            if (mag > PUSH_SPEED_MAX) mag = PUSH_SPEED_MAX;
            outX = sign * mag;
        }
        if (inPushZ) {
            double excess = (fabs(offZ) - PUSH_ENTER_RAD) / (1.0 - PUSH_ENTER_RAD);
            excess = excess < 0.0 ? 0.0 : (excess > 1.0 ? 1.0 : excess);
            double sign = offZ >= 0.0 ? 1.0 : -1.0;
            double base = fabs(entryVelZ) > 0.01 ? fabs(entryVelZ) : PUSH_SPEED_BASE;
            double mag = base + excess * (PUSH_SPEED_MAX - base);
            if (mag > PUSH_SPEED_MAX) mag = PUSH_SPEED_MAX;
            outY = sign * mag * PUSH_VERTICAL_SCALE;   // up/down at reduced speed
        }
        return inPush;
    }
};

static int16_t ToStick(double v) {
    if (v > 1.0) v = 1.0;
    if (v < -1.0) v = -1.0;
    return (int16_t)(v * 32767.0);
}

static void ApplyForces(double y, double z,
    const AxisState& axY, const AxisState& axZ,
    double velY, double velZ,
    double rumX, double rumY, double rumZ,
    double dt)
{
    double offY = axY.Offset(y), offZ = axZ.Offset(z);
    double r = sqrt(offY * offY + offZ * offZ);
    double forceY = 0.0, forceZ = 0.0;

    //Spring force bounding box
    if (r > FORCE_SPRING_START) {
        double dirY = offY / r, dirZ = offZ / r;
        double t = (r - FORCE_SPRING_START) / (FORCE_MAX_RAD - FORCE_SPRING_START);
        t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
        double mag = pow(t, FORCE_EXPONENT) * FORCE_MAX_N;
        double vOut = velY * dirY + velZ * dirZ; // outward velocity component
        // Bidirectional radial damping. Previously damping only applied when vOut > 0,
        // leaving the wall a completely UNDAMPED stiff spring on the inward half of
        // every oscillation cycle — the classic buzzing virtual wall (a sampled spring
        // injects a little energy each cycle; one-sided damping only removes it half
        // the time). Base FORCE_DAMPING now acts on both signs of vOut: it resists
        // compression on the way out (adds inward force) and resists retreat on the
        // way in (reduces the push-back), which is what removes the oscillation energy.
        // The outward-only proximity/velGate ENHANCED damping is unchanged.
        double dynamicDamp = FORCE_DAMPING;
        if (vOut > 0.0) {
            double proximity = (r - FORCE_SPRING_START) / (PUSH_ENTER_RAD - FORCE_SPRING_START);
            proximity = proximity < 0.0 ? 0.0 : (proximity > 1.0 ? 1.0 : proximity);

            // Gate: only apply enhanced damping above a velocity threshold
            // (DAMP_VEL_MIN/MAX are now config globals)
            double velMag = sqrt(velY * velY + velZ * velZ);
            double velGate = (velMag - DAMP_VEL_MIN) / (DAMP_VEL_MAX - DAMP_VEL_MIN);
            velGate = velGate < 0.0 ? 0.0 : (velGate > 1.0 ? 1.0 : velGate);
            velGate = velGate * velGate * (3.0 - 2.0 * velGate); // smoothstep

            dynamicDamp = FORCE_DAMPING * (1.0 + proximity * 2.0 * velGate);
        }
        mag += vOut * dynamicDamp;
        if (mag < 0.0) mag = 0.0;   // damping may reduce the wall force, never reverse it
        forceY = -dirY * mag;
        forceZ = -dirZ * mag;
    }

    // ── Boundary detent — the wall at the velocity-zone edge ─────────────────────
    // PURE POSITION FUNCTION. Earlier versions gated the ridge on radial-velocity
    // direction (arm moving out, suppress moving in); a tangential slide along the
    // wall has no stable radial sign, so any velocity-direction gate MUST strobe the
    // full ridge force on/off there — that was the rumble, and no amount of signal
    // filtering fixes a binary gate keyed to a signal that wanders across zero.
    // The case the gate existed for (post-pop re-entry dragging inward across the
    // band) is handled by the edgePopped spatial latch below, so the gate is gone.
    // A position-only force cannot rumble under tangential motion: sliding the wall
    // is now a constant, full-strength ridge. Inward passes that never popped get a
    // gentle inward assist toward center (detent-valley behavior).
    static bool edgePopped = false;
    static bool edgePoppedPrev = false;
    if (r >= PUSH_ENTER_RAD) edgePopped = true;
    else if (r < PUSH_ENTER_RAD - EDGE_HYST) edgePopped = false;

    double detentStart = PUSH_ENTER_RAD - DETENT_WIDTH;        // ridge begins here
    if (!edgePopped && r > detentStart && r < PUSH_ENTER_RAD && r > 0.0001) {
        double dirY = offY / r, dirZ = offZ / r;
        double u = (r - detentStart) / DETENT_WIDTH;           // 0 at ridge start, 1 at push threshold
        u = u * u * (3.0 - 2.0 * u);                           // smoothstep — gentle build
        double lip = u * DETENT_PEAK_N;
        forceY += -dirY * lip;                                 // resist outward; releases past threshold = the "pop"
        forceZ += -dirZ * lip;
    }

    // ── Exit pump — a brief inward kick when leaving the push border ──────────────
    // Fires on the pop latch's release transition (r retreating EDGE_HYST back
    // inside), so it lands once per genuine excursion instead of machine-gunning
    // on border wobble like the old bare r >= PUSH_ENTER_RAD comparison did.
    static double pumpEnv = 0.0;
    if (edgePoppedPrev && !edgePopped) pumpEnv = 1.0;          // genuine exit → fire once
    edgePoppedPrev = edgePopped;
    if (pumpEnv > 0.01 && r > 0.0001) {
        double dirY = offY / r, dirZ = offZ / r;
        double kick = PUMP_PEAK_N * pumpEnv;
        forceY += -dirY * kick;                                // inward kick toward center
        forceZ += -dirZ * kick;
    }
    pumpEnv *= pow(PUMP_DECAY, dt * 1000.0);
    if (pumpEnv < 0.01) pumpEnv = 0.0;

    forceY += rumY;
    forceZ += rumZ;

    // Friction cancellation
    // NOTE: previously this block re-declared FRICTION_VEL_MIN/MAX as static const
    // locals (0.003 / 0.08), silently shadowing the globals — the global values were
    // never in effect. The shadows are removed; the globals (defaults set to the
    // previously-effective values) now actually control this, including via config.
    double velMag = sqrt(velY * velY + velZ * velZ);
    if (velMag > FRICTION_VEL_MIN) {
        double fadeIn = fmin((velMag - FRICTION_VEL_MIN) / (FRICTION_VEL_FULL - FRICTION_VEL_MIN), 1.0);
        double fadeOut = fmax(1.0 - (velMag - FRICTION_VEL_FULL) / (FRICTION_VEL_MAX - FRICTION_VEL_FULL), 0.0);
        double blend = fadeIn * fadeOut;
        forceY += (velY / velMag) * FRICTION_CANCEL * blend;
        forceZ += (velZ / velMag) * FRICTION_CANCEL * blend;
    }

    //Coulomb breakaway
    if (velMag > STICTION_VEL_ON) {
        forceY += (velY / velMag) * STICTION_BREAK_N;
        forceZ += (velZ / velMag) * STICTION_BREAK_N;
    }

    // Viscous cancellation — velocity-PROPORTIONAL assist (Coulomb can't cancel drag
    // that grows with speed). Direction is automatic from the velocity sign. Capped
    // at VISCOUS_MAX_N because this is deliberate negative damping: past the real
    // drag coefficient it becomes energy injection.
    if (VISCOUS_CANCEL > 0.0 && velMag > STICTION_VEL_ON) {
        double vy = velY * VISCOUS_CANCEL;
        double vz = velZ * VISCOUS_CANCEL;
        double vmag = sqrt(vy * vy + vz * vz);
        if (vmag > VISCOUS_MAX_N) { double s = VISCOUS_MAX_N / vmag; vy *= s; vz *= s; }
        forceY += vy;
        forceZ += vz;
    }

    // Stiction dither — small circular force at DITHER_HZ, full strength at rest and
    // smoothstep-faded to zero by DITHER_VEL_FADE. Keeps the sleds in kinetic friction
    // so there is no static re-grip at reversals or on the first mm of a correction.
    // Circular (sin on Y, cos on Z) so it never biases a direction.
    if (DITHER_N > 0.0) {
        static double ditherPhase = 0.0;
        ditherPhase += dt * DITHER_HZ * 2.0 * 3.14159265;
        if (ditherPhase > 2.0 * 3.14159265 * 1000.0) ditherPhase -= 2.0 * 3.14159265 * 1000.0;
        double fade = 1.0 - velMag / DITHER_VEL_FADE;
        fade = fade < 0.0 ? 0.0 : (fade > 1.0 ? 1.0 : fade);
        fade = fade * fade * (3.0 - 2.0 * fade);
        if (fade > 0.0) {
            forceY += DITHER_N * fade * sin(ditherPhase);
            forceZ += DITHER_N * fade * cos(ditherPhase);
        }
    }

    // Cap X and YZ independently
    // X = recoil/rumble push, YZ = spring + lateral rumble
    double yzMag = sqrt(forceY * forceY + forceZ * forceZ);
    if (yzMag > FORCE_YZ_CAP_N) {
        double s = FORCE_YZ_CAP_N / yzMag;
        forceY *= s;
        forceZ *= s;
    }
    if (rumX > FORCE_MAX_N) rumX = FORCE_MAX_N;
    if (rumX < -FORCE_MAX_N) rumX = -FORCE_MAX_N;

    dhdSetForce(rumX, forceY, forceZ);
}

// ── Main ───────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    InitializeCriticalSection(&g_recoilCS);

    printf("FalconJoyForce - Falcon -> Pico 2W (Serial) -> Xbox Controller\n");
    printf("Build: %s %s\n\n", __DATE__, __TIME__);

    // Optional per-title profile: first argument overrides the config path
    if (argc > 1) {
        strncpy(g_configPath, argv[1], sizeof(g_configPath) - 1);
        g_configPath[sizeof(g_configPath) - 1] = 0;
    }

    // Load config BEFORE anything uses the tunables (serial port, gravity, etc.)
    LoadConfig(g_configPath);
    ApplyDerivedConfig();

    if (BRACE_BUTTON >= 1 && BRACE_BUTTON <= 4)
        printf("Brace: button %d (consumed from controller output), %s mode, vel x%.2f, push x%.2f\n",
            BRACE_BUTTON, BRACE_TOGGLE ? "toggle" : "hold", BRACE_VEL_SCALE, BRACE_PUSH_SCALE);
    else
        printf("Brace: disabled\n");

    printf("Opening %s at %lu baud...\n", SERIAL_PORT, (unsigned long)SERIAL_BAUD);
    if (!SerialOpen(SERIAL_PORT, SERIAL_BAUD)) {
        printf("ERROR: Cannot open %s\n", SERIAL_PORT);
        printf("Check Device Manager -> Ports for the correct COM number.\n");
        getchar(); return -1;
    }
    printf("Serial port open.\n");

    HANDLE hThread = CreateThread(NULL, 0, SerialReaderThread, NULL, 0, NULL);
    if (!hThread) {
        printf("ERROR: Cannot start reader thread.\n");
        SerialClose(); getchar(); return -1;
    }

    if (dhdOpen() < 0) {
        printf("ERROR: Cannot open Falcon.\n");
        SerialClose(); getchar(); return -1;
    }

    timeBeginPeriod(1);
    // Enable gravity compensation
    dhdSetGravityCompensation(DHD_ON);
    dhdSetStandardGravity(9.81 * GRAVITY_COMP_SCALE);

    printf("Falcon ready. SDK %s\n\n", dhdGetSDKVersionStr());
    printf("Serial: %s   Push zone: %.0f%%   Force max rad: %.2f\n",
        SERIAL_PORT, PUSH_ENTER_RAD * 100.0, FORCE_MAX_RAD);
    printf("F8=reload config  F9=quit  F10=debug  F11=recalibrate\n\n");

    AxisState   axY, axZ;
    PushState2D push;

    // ── High-resolution timing ─────────────────────────────────────────────
    // GetTickCount is 1ms-quantized: at ~1kHz that makes dt read 0/1/2ms at random,
    // corrupting every frequency-based IIR filter. QPC gives sub-microsecond dt.
    // GetTickCount() is kept for coarse ms timestamps (rumble freshness, UI).
    LARGE_INTEGER qpf;   QueryPerformanceFrequency(&qpf);
    LARGE_INTEGER qLast; QueryPerformanceCounter(&qLast);
    LARGE_INTEGER qDeadline = qLast;
    LONGLONG periodTicks = (UPDATE_RATE_MS > 0) ? (qpf.QuadPart * UPDATE_RATE_MS) / 1000 : 0;

    double rumblePhase = 0.0;
    DWORD  lastStats = GetTickCount();
    DWORD  lastKeyCheck = 0;      // GetAsyncKeyState throttle
    int    hz = 0;
    bool   btn1WasHeld = false;
    g_btn1Released = GetTickCount();

    double  serialAccumMs = 0.0;   // controller packet decimation
    uint8_t lastSentBtnMask = 0;

    bool   recoilActive = false;
    bool   wasRecoilActive = false;

    while (true) {
        LARGE_INTEGER qNow; QueryPerformanceCounter(&qNow);
        double dt = (double)(qNow.QuadPart - qLast.QuadPart) / (double)qpf.QuadPart;
        qLast = qNow;
        if (dt <= 0.0) dt = 0.0005;
        if (dt > 0.05) dt = 0.01;
        DWORD now = GetTickCount();

        double x, y, z;
        if (dhdGetPosition(&x, &y, &z) < 0) { printf("Lost Falcon\n"); break; }

        axY.UpdateVelocity(y, dt);
        axZ.UpdateVelocity(z, dt);
        axY.UpdateSlowRef(y, dt);
        axZ.UpdateSlowRef(z, dt);
        axY.UpdateReach(y);
        axZ.UpdateReach(z);

        double offY = axY.Offset(y), offZ = axZ.Offset(z);
        double stickX = 0.0, stickY = 0.0;
        bool inPush = push.Update(offY, offZ, axY.smoothVel, axZ.smoothVel, stickX, stickY);

        // ── Push zone reversal damping ─────────────────────────────────────
        static bool wasInPush = false;
        if (inPush) {
            double dotY = -(offY * axY.smoothVel);
            double dotZ = -(offZ * axZ.smoothVel);
            double dotVelToCenter = dotY + dotZ;
            if (dotVelToCenter > 0.0) {
                double dampTrigger = dotVelToCenter * PUSH_DAMP_COEFF;
                if (dampTrigger > g_pushDamp) g_pushDamp = dampTrigger;
            }
        }
        else {
            if (wasInPush) g_pushDamp = 1.0;
        }
        wasInPush = inPush;
        g_pushDamp -= g_pushDamp * PUSH_DAMP_DECAY * dt;
        if (g_pushDamp < 0.0) g_pushDamp = 0.0;
        if (g_pushDamp > 1.0) g_pushDamp = 1.0;

        // ── Recoil velocity bleed ──────────────────────────────────────────────
        // Prevents physical Falcon settling during recoil tail from reaching stick output
        if (g_recoilFiring) {
            double bleedStrength = (g_recoilForce / RECOIL_MAG_SCALE);
            bleedStrength = bleedStrength < 0.0 ? 0.0 : (bleedStrength > 1.0 ? 1.0 : bleedStrength);
            // Bias: specifically gate downward (negative Z) velocity harder than upward
            // since gravity is the primary cause of the tail drift
            double bleedY = 1.0 - bleedStrength * BLEED_Y_SCALE;
            double bleedZ = (axZ.smoothVel < 0.0)
                ? 1.0 - bleedStrength * BLEED_Z_DOWN   // stronger bleed on downward motion
                : 1.0 - bleedStrength * BLEED_Z_UP;    // lighter on upward
            axY.smoothVel *= bleedY;
            axZ.smoothVel *= bleedZ;
        }

        axY.UpdateFlick(dt);   // bank/pay-out fast-motion carry (uses post-bleed velocity)
        axZ.UpdateFlick(dt);

        if (!inPush) {
            auto evalZone = [](double v, double sens, double crv) -> double {
                double scaled = fabs(v) * sens;
                if (scaled > 1.0) scaled = 1.0;
                return pow(scaled, crv);
            };
            auto curve = [&](double v) -> double {
                if (fabs(v) < VEL_DEADZONE) return 0.0;
                double sign = v > 0.0 ? 1.0 : -1.0;
                double speed = fabs(v);

                double ramp = (speed - VEL_DEADZONE) / SOFT_RAMP;
                ramp = ramp < 0.0 ? 0.0 : (ramp > 1.0 ? 1.0 : ramp);
                ramp = ramp * ramp * (3.0 - 2.0 * ramp);

                double vlow = evalZone(v, VEL_VLOW_SENS, VEL_VLOW_CURVE);
                double lo = evalZone(v, VEL_LOW_SENS, VEL_LOW_CURVE);
                double hi = pow(fmin(speed * VEL_HIGH_SENS, 1.0), VEL_HIGH_CURVE);
                if (hi > 1.0) hi = 1.0;

                // Blend vlow -> lo
                double tVlow = (speed - VEL_BLEND_VLOW) / (VEL_BLEND_LOW - VEL_BLEND_VLOW);
                tVlow = tVlow < 0.0 ? 0.0 : (tVlow > 1.0 ? 1.0 : tVlow);
                tVlow = tVlow * tVlow * (3.0 - 2.0 * tVlow);
                double lowBlended = vlow + (lo - vlow) * tVlow;

                // Blend lowBlended -> hi
                double tHi = (speed - VEL_BLEND_LOW) / (VEL_BLEND_HIGH - VEL_BLEND_LOW);
                tHi = tHi < 0.0 ? 0.0 : (tHi > 1.0 ? 1.0 : tHi);
                tHi = tHi * tHi * (3.0 - 2.0 * tHi);

                return sign * (lowBlended * (1.0 - tHi) + hi * tHi) * ramp;
            };

            stickX = curve(axY.smoothVel);
            stickY = curve(axZ.smoothVel);

            // Position-error blend — kicks in when velocity signal is unreliable
            double velMag = sqrt(axY.smoothVel * axY.smoothVel + axZ.smoothVel * axZ.smoothVel);
            double posBlendRaw = 1.0 - velMag / posBlendMax;
            posBlendRaw = posBlendRaw < 0.0 ? 0.0 : (posBlendRaw > 1.0 ? 1.0 : posBlendRaw);
            posBlendRaw = posBlendRaw * posBlendRaw * (3.0 - 2.0 * posBlendRaw);

            static double posBlendSm = 0.0;  // slew-limited — a one-frame velMag dip can't snap this
            double aPB = 1.0 - exp(-2.0 * 3.14159265 * POSBLEND_SLEW_HZ * dt);
            posBlendSm += aPB * (posBlendRaw - posBlendSm);
            double posBlend = posBlendSm;

            if (posBlend > 0.0) {
                double pcX = axY.PosError(y) * VEL_VLOW_POS_SCALE;
                double pcY = axZ.PosError(z) * VEL_VLOW_POS_SCALE;
                pcX = pcX < -1.0 ? -1.0 : (pcX > 1.0 ? 1.0 : pcX);
                pcY = pcY < -1.0 ? -1.0 : (pcY > 1.0 ? 1.0 : pcY);
                stickX = stickX * (1.0 - posBlend) + pcX * posBlend;
                stickY = stickY * (1.0 - posBlend) + pcY * posBlend;
            }

            stickX += axY.flickCarry;   // pay out banked fast-motion as a short carry tail
            stickY += axZ.flickCarry;   // (ToStick clamps the sum to ±1)
            stickY *= VEL_VERTICAL_SCALE;   // up/down at reduced speed (velocity zone)
        }

        rumblePhase += dt * 80.0 * 2.0 * 3.14159265;
        float rL = g_rumbleLarge, rS = g_rumbleSmall;

        // ── Braced aim mode ────────────────────────────────────────────────
        // Scales down aim output for precision. Applied after both zones so it
        // covers velocity output, flick carry, position blend, and push speed.
        static bool braceToggled = false;
        static bool braceBtnWas = false;
        bool braced = false;
        if (BRACE_BUTTON >= 1 && BRACE_BUTTON <= 4) {
            bool braceBtnHeld = (dhdGetButton(BRACE_BUTTON - 1) > 0);
            if (BRACE_TOGGLE) {
                if (braceBtnHeld && !braceBtnWas) braceToggled = !braceToggled;
                braced = braceToggled;
            }
            else {
                braced = braceBtnHeld;
            }
            braceBtnWas = braceBtnHeld;
        }
        else {
            braceToggled = false;   // disabling the feature clears a latched toggle
        }
        if (braced) {
            double s = inPush ? BRACE_PUSH_SCALE : BRACE_VEL_SCALE;
            stickX *= s;
            stickY *= s;
        }
        static bool braceWasActive = false;
        if (braced != braceWasActive) {
            printf("\nBraced aim %s (vel x%.2f, push x%.2f)\n",
                braced ? "ON" : "OFF", BRACE_VEL_SCALE, BRACE_PUSH_SCALE);
            braceWasActive = braced;
        }

        // ── Button 1 recoil logic ──────────────────────────────────
        bool btn1held = (dhdGetButton(0) > 0);
        if (btn1WasHeld && !btn1held) g_btn1Released = now;
        btn1WasHeld = btn1held;
        recoilActive = btn1held || (now - g_btn1Released < RECOIL_WINDOW_MS);

        if (recoilActive && !wasRecoilActive) g_recoilDampEnv = 1.0;
        wasRecoilActive = recoilActive;

        g_recoilDampEnv -= g_recoilDampEnv * RECOIL_DAMP_DECAY * dt;
        if (g_recoilDampEnv < 0.0) g_recoilDampEnv = 0.0;

        double rumX = 0.0, rumY = 0.0, rumZ = 0.0;

        if (recoilActive) {
            double liveMag = (g_rumbleLargePeak * RUMBLE_LARGE_SCALE + g_rumbleSmallPeak * RUMBLE_SMALL_SCALE);
            // Stale held-peak guard: g_rumbleLargePeak/SmallPeak are raw and never
            // self-decay, so after firing stops a latched peak keeps sustainingRumble true
            // and re-fires the recoil (the ratchet on release). Zero it ONLY while the
            // trigger is released — while the trigger is HELD we must never zero liveMag,
            // or a momentary packet gap would drop the sustained-fire hold.
            if (!btn1held && (now - g_lastRumbleTime) >= RUMBLE_FRESH_MS) liveMag = 0.0;
            double queuePeak = 0.0, nextPeak = 0.0;
            while (RecoilDequeue(nextPeak))
                if (nextPeak > queuePeak) queuePeak = nextPeak;

            if (queuePeak > 0.0) {
                if (!g_recoilFiring) {
                    g_recoilForce = liveMag;
                    g_recoilPeak = liveMag;
                    g_recoilAttack = (RECOIL_ATTACK_SEC > 0.0) ? 0.0 : 1.0;
                    g_recoilFiring = true;
                    g_rapidShotCount = 0;
                    g_recoilYZForce = 0.0;
                    double magT = liveMag / RECOIL_MAG_SCALE;
                    magT = magT < 0.0 ? 0.0 : (magT > 1.0 ? 1.0 : magT);
                    g_recoilDecay = RECOIL_DECAY_MIN + magT * (RECOIL_DECAY_MAX - RECOIL_DECAY_MIN);
                }
                else {
                    g_rapidShotCount++;
                    if (liveMag > g_recoilForce) g_recoilForce = liveMag;
                    g_recoilPeak = g_recoilForce;
                    g_recoilYZForce = liveMag * RECOIL_YZ_SCALE;
                    double magT = liveMag / RECOIL_MAG_SCALE;
                    magT = magT < 0.0 ? 0.0 : (magT > 1.0 ? 1.0 : magT);
                    g_recoilDecay = RECOIL_DECAY_MIN + magT * (RECOIL_DECAY_MAX - RECOIL_DECAY_MIN);
                }
            }

            bool sustainingRumble = (liveMag > RECOIL_SUSTAIN_THRESHOLD);
            if (sustainingRumble) {
                if (liveMag > g_recoilForce) { g_recoilForce = liveMag; g_recoilPeak = liveMag; }
                if (!g_recoilFiring && liveMag > 0.01) {
                    g_recoilForce = liveMag; g_recoilPeak = liveMag;
                    g_recoilFiring = true;    g_recoilAttack = 1.0;
                }
            }

            double liveYZMag = 0.0;
            if (g_rapidShotCount > 0 || sustainingRumble)
                liveYZMag = liveMag * RECOIL_YZ_SCALE;
            if (liveYZMag > g_recoilYZForce) g_recoilYZForce = liveYZMag;
            g_recoilYZForce *= pow(RUMBLE_DECAY, dt * 1000.0);   // dt-based (was per-frame)
            if (g_recoilYZForce < 0.01) g_recoilYZForce = 0.0;

            if (g_recoilYZForce > 0.01 && (now - g_lastYZDirChange) > RECOIL_DIR_CHANGE_MS) {
                g_lastYZDirChange = now;
                double angle = ((rand() % 1000) / 1000.0) * 2.0 * 3.14159265;
                g_recoilYZTgtY = cos(angle);   // set target; actual dir slews toward it below
                g_recoilYZTgtZ = sin(angle);
            }

            if (g_recoilFiring) {
                double envelope = 1.0;
                if (RECOIL_ATTACK_SEC > 0.0) {
                    g_recoilAttack += dt / RECOIL_ATTACK_SEC;
                    if (g_recoilAttack > 1.0) g_recoilAttack = 1.0;
                    envelope = g_recoilAttack;
                }
                rumX = g_recoilPeak * envelope * RECOIL_X_SCALE * 2.0;
                // lateral texture (rumY/rumZ) rendered by the consolidated slewed block below

                if (!sustainingRumble && g_recoilAttack >= 1.0) {
                    double frameDecay = pow(g_recoilDecay, dt * 1000.0);
                    g_recoilForce *= frameDecay;
                    g_recoilPeak *= frameDecay;
                    if (g_recoilForce < 0.01) {
                        g_recoilForce = 0.0;
                        g_recoilFiring = false;
                        g_rapidShotCount = 0;
                    }
                }
            }
        }
        else {
            if (!g_recoilFiring) {
                g_recoilForce = 0.0;
                g_recoilPeak = 0.0;
                g_recoilAttack = 0.0;
            }
            g_recoilYZForce = 0.0;
            g_rapidShotCount = 0;
            double tmp;
            while (RecoilDequeue(tmp)) {}
        }

        if (!recoilActive && g_recoilFiring) {
            double frameDecay = pow(g_recoilDecay, dt * 1000.0);
            g_recoilForce *= frameDecay;
            g_recoilPeak *= frameDecay;
            rumX = g_recoilPeak * RECOIL_X_SCALE * 2.0;
            if (g_recoilForce < 0.01) { g_recoilForce = 0.0; g_recoilFiring = false; }
        }

        // ── Recoil push release shaping ──────────────────────────────────────────────
        // The raw recoil force sawtooths as it ramps down: each incoming tail packet
        // re-bumps g_recoilForce, which then collapses fast before the next arrives — that
        // sawtooth is the buzz. Snap UP instantly so the kick and sustained hold stay
        // sharp, but low-pass on the way DOWN so the release tracks the rumble's decay
        // envelope as one clean ramp. RECOIL_RELEASE_HZ sets sharper (higher) vs smoother.
        static double g_recoilXOut = 0.0;
        if (rumX >= g_recoilXOut) {
            g_recoilXOut = rumX;                                   // rising → keep the sharp kick / hold
        }
        else {
            double aRel = 1.0 - exp(-2.0 * 3.14159265 * RECOIL_RELEASE_HZ * dt);
            g_recoilXOut += aRel * (rumX - g_recoilXOut);         // falling → smooth, buzz-free ramp down
        }
        if (fabs(g_recoilXOut) < 0.01) g_recoilXOut = 0.0;        // reach true 0 so ambient can resume
        rumX = g_recoilXOut;

        // Hold ambient off through the ENTIRE recoil rumble tail, however long it runs.
        // A recoil opens a "tail session" that stays open until the incoming rumble has
        // decayed below AMBIENT_TAIL_QUIET or stopped arriving. This adapts to shot size —
        // larger shots have bigger/longer tails — which a fixed time window did not, which
        // is why the buzz survived on larger shots.
        {
            double liveInNow = (g_rumbleLargePeak * RUMBLE_LARGE_SCALE + g_rumbleSmallPeak * RUMBLE_SMALL_SCALE);
            bool   rumbleFresh = (now - g_lastRumbleTime) < RUMBLE_FRESH_MS;
            if (recoilActive || g_recoilFiring)                      g_recoilTailActive = true;
            else if (!rumbleFresh || liveInNow < AMBIENT_TAIL_QUIET) g_recoilTailActive = false;
        }

        if (rumX == 0.0) {
            static double rumDirY = 1.0, rumDirZ = 0.0;   // current (slewed) direction
            static double tgtDirY = 1.0, tgtDirZ = 0.0;   // target direction
            static double rumMagSm = 0.0;                 // smoothed magnitude
            static DWORD  lastDirChange = 0;
            // envelope constants are now config globals: AMBIENT_DIR_CHANGE_MS,
            // AMBIENT_DIR_SLEW_HZ, AMBIENT_ATTACK_HZ, AMBIENT_RELEASE_HZ

            double magTarget = (rL * AMBIENT_LARGE_SCALE + rS * AMBIENT_SMALL_SCALE);

            // Hold ambient off for the whole recoil tail session — the smoother below then
            // keeps ambient faded out instead of buzzing on the tail. Genuine ambient with
            // no recent firing is unaffected.
            if (g_recoilTailActive) magTarget = 0.0;

            // pick a new target direction occasionally
            if (now - lastDirChange > AMBIENT_DIR_CHANGE_MS) {
                lastDirChange = now;
                double angle = ((rand() % 1000) / 1000.0) * 2.0 * 3.14159265;
                tgtDirY = cos(angle);
                tgtDirZ = sin(angle);
            }

            // slew the actual direction toward the target, then renormalize
            double aDir = 1.0 - exp(-2.0 * 3.14159265 * AMBIENT_DIR_SLEW_HZ * dt);
            rumDirY += aDir * (tgtDirY - rumDirY);
            rumDirZ += aDir * (tgtDirZ - rumDirZ);
            double dlen = sqrt(rumDirY * rumDirY + rumDirZ * rumDirZ);
            if (dlen > 1e-6) { rumDirY /= dlen; rumDirZ /= dlen; }

            // attack/release low-pass on magnitude
            double mhz = (magTarget > rumMagSm) ? AMBIENT_ATTACK_HZ : AMBIENT_RELEASE_HZ;
            double aMag = 1.0 - exp(-2.0 * 3.14159265 * mhz * dt);
            rumMagSm += aMag * (magTarget - rumMagSm);

            if (rumMagSm > 0.001) {
                rumY = rumDirY * rumMagSm;
                rumZ = rumDirZ * rumMagSm;
            }
        }

        // ── Recoil lateral texture: only during the STRONG part of the rumble ───────
        // Incoming rumble per shot is a sharp spike then a long weak decaying tail of
        // packets. Rendering the tail directly turned every little packet into a separate
        // lateral nudge = the jagged rumble "coming down." A hysteresis gate keeps the
        // texture on only while the rumble is genuinely strong (HI to turn on, LO to turn
        // off), so it bursts with the kick and cuts before the weak tail. The level is
        // smoothed while active (de-jag) and hard-cut once the gate fails (clean release).
        {
            double liveIn = (g_rumbleLargePeak * RUMBLE_LARGE_SCALE + g_rumbleSmallPeak * RUMBLE_SMALL_SCALE);
            bool   rumbleIncoming = (now - g_lastRumbleTime) < RUMBLE_FRESH_MS;

            static bool yzOn = false;
            if (!recoilActive || !rumbleIncoming)   yzOn = false;            // outside fire / no fresh rumble → off
            else if (liveIn > RECOIL_YZ_GATE_HI)    yzOn = true;             // strong rumble → on
            else if (liveIn < RECOIL_YZ_GATE_LO)    yzOn = false;            // dropped into the weak tail → off

            double yzTarget = yzOn ? liveIn * RECOIL_YZ_SCALE : 0.0;
            double yzHz = (yzTarget > g_recoilYZLevel) ? RECOIL_YZ_SMOOTH_HZ : RECOIL_YZ_CUT_HZ;
            double aMag = 1.0 - exp(-2.0 * 3.14159265 * yzHz * dt);
            g_recoilYZLevel += aMag * (yzTarget - g_recoilYZLevel);
            if (g_recoilYZLevel < 0.001) g_recoilYZLevel = 0.0;

            if (g_recoilYZLevel > 0.0) {
                double aDir = 1.0 - exp(-2.0 * 3.14159265 * RECOIL_YZ_DIR_SLEW_HZ * dt);
                g_recoilYZDirY += aDir * (g_recoilYZTgtY - g_recoilYZDirY);
                g_recoilYZDirZ += aDir * (g_recoilYZTgtZ - g_recoilYZDirZ);
                double yzlen = sqrt(g_recoilYZDirY * g_recoilYZDirY + g_recoilYZDirZ * g_recoilYZDirZ);
                if (yzlen > 1e-6) { g_recoilYZDirY /= yzlen; g_recoilYZDirZ /= yzlen; }
                rumY += g_recoilYZDirY * g_recoilYZLevel;
                rumZ += g_recoilYZDirZ * g_recoilYZLevel;
            }
        }

        // dt-based rumble decay (was per-frame: feel changed with loop rate).
        // RUMBLE_DECAY is now the per-millisecond retention factor.
        {
            float rumbleFrameDecay = (float)pow(RUMBLE_DECAY, dt * 1000.0);
            g_rumbleLarge = rL * rumbleFrameDecay;
            g_rumbleSmall = rS * rumbleFrameDecay;
        }

        // ── Button 2: constant X preload to ease small corrections ───────────────────
        if (dhdGetButton(1) > 0) rumX += BTN2_X_SIGN * BTN2_X_FORCE;

        ApplyForces(y, z, axY, axZ, axY.smoothVel, axZ.smoothVel, rumX, rumY, rumZ, dt);


        // ── Button mask ────────────────────────────────────────────────────
        uint8_t btnMask = 0;
        for (int i = 0; i < 4; i++)
            if (dhdGetButton(i) > 0) btnMask |= (1u << i);
        if (BRACE_BUTTON >= 1 && BRACE_BUTTON <= 4)
            btnMask &= (uint8_t)~(1u << (BRACE_BUTTON - 1));   // brace button is consumed, never sent

        // ── Send controller packet (decimated) ─────────────────────────────
        // The haptic loop runs ~1kHz but the Pico/Xbox side samples far slower;
        // writing 7 bytes over 115200-baud serial every frame both risks blocking
        // in WriteFile and pushes the link near saturation. Send every
        // SERIAL_SEND_MS instead — but flush immediately on any button change so
        // button latency is unaffected.
        double stickScale = 1.0 - g_pushDamp;
        if (stickScale < 0.0) stickScale = 0.0;
        if (recoilActive)
            stickScale *= 1.0 - (1.0 - RECOIL_AIM_DAMP) * g_recoilDampEnv;

        serialAccumMs += dt * 1000.0;
        if (serialAccumMs >= SERIAL_SEND_MS || btnMask != lastSentBtnMask) {
            serialAccumMs = 0.0;
            int16_t lx = ToStick(stickX * stickScale * (INVERT_X ? -1.0 : 1.0));
            int16_t ly = ToStick(stickY * stickScale * (INVERT_Y ? -1.0 : 1.0));
            SendControllerPacket(lx, ly, btnMask);
            lastSentBtnMask = btnMask;
        }

        // ── Status display ─────────────────────────────────────────────────
        hz++;
        if (now - lastStats >= 1000) {
            int qcount = (g_recoilQHead - g_recoilQTail + RECOIL_QUEUE_SIZE) % RECOIL_QUEUE_SIZE;
            double r = sqrt(offY * offY + offZ * offZ);
            if (g_debugMode) {
                printf("\nY: ctr=%+.4f half=%.4f off=%+.2f vel=%+.4f\n",
                    axY.estCenter, axY.halfRange, offY, axY.smoothVel);
                printf("Z: ctr=%+.4f half=%.4f off=%+.2f vel=%+.4f\n",
                    axZ.estCenter, axZ.halfRange, offZ, axZ.smoothVel);
                printf("r=%.3f  push=%s  damp=%.2f  stk=(%.2f,%.2f)  rbl=%.2f/%.2f  rcl=%.2f  q=%d  btn=%X\n",
                    r, inPush ? "YES" : "no", g_pushDamp, stickX, stickY,
                    rL, rS, g_recoilForce, qcount, btnMask);
                if (AGC_ENABLE) {
                    double tol = (AGC_TOLERANCE < 1.0) ? 1.0 : AGC_TOLERANCE;
                    double lo = AGC_TARGET / tol, hi = AGC_TARGET * tol;
                    bool inBand = (g_agcAvg >= lo && g_agcAvg <= hi);
                    double eff = g_agcAvg < lo ? lo : (g_agcAvg > hi ? hi : g_agcAvg);
                    double g = (g_agcAvg > 1e-6) ? eff / g_agcAvg : 1.0;
                    g = g < AGC_GAIN_MIN ? AGC_GAIN_MIN : (g > AGC_GAIN_MAX ? AGC_GAIN_MAX : g);
                    printf("agc: avg=%.3f band=%.3f..%.3f %s level-gain=%.2f\n",
                        g_agcAvg, lo, hi, inBand ? "IN-BAND (raw)" : "correcting", g);
                }
            }
            else {
                int col = (int)((offY + 1.0) / 2.0 * 8.0 + 0.5);
                col = col < 0 ? 0 : (col > 8 ? 8 : col);
                printf("\r%4d Hz %s r=%.2f %s  damp=%.2f  [", hz, braced ? "BRC" : "   ", r, inPush ? "PUSH" : "    ", g_pushDamp);
                for (int c = 0; c <= 8; c++) printf(c == col ? "O" : ".");
                printf("]  stk=(%.2f,%.2f)  vel=%.3f/%.3f  rbl=%.2f/%.2f  rcl=%.2f  q=%d  btn=%X   ",
                    stickX, stickY, axY.smoothVel, axZ.smoothVel, rL, rS, g_recoilForce, qcount, btnMask);
                fflush(stdout);
            }
            hz = 0; lastStats = now;
        }

        // ── Keyboard (throttled — 3 GetAsyncKeyState syscalls/frame at 1kHz is waste) ──
        if (now - lastKeyCheck >= 25) {
            lastKeyCheck = now;
            if (GetAsyncKeyState(VK_F9) & 0x8000) break;
            if (GetAsyncKeyState(VK_F8) & 0x8000) {
                printf("\nReloading %s...\n", g_configPath);
                LoadConfig(g_configPath);
                ApplyDerivedConfig();
                dhdSetStandardGravity(9.81 * GRAVITY_COMP_SCALE);   // re-apply gravity comp
                periodTicks = (UPDATE_RATE_MS > 0) ? (qpf.QuadPart * UPDATE_RATE_MS) / 1000 : 0;
                QueryPerformanceCounter(&qDeadline);                // reset pacing baseline
                printf("(HALF_RANGE_MAX / serial port changes need F11 recalibrate / restart)\n");
                Sleep(200);
            }
            if (GetAsyncKeyState(VK_F10) & 0x8000) {
                g_debugMode = !g_debugMode;
                printf("\nDebug %s\n", g_debugMode ? "ON" : "OFF");
                Sleep(200);
            }
            if (GetAsyncKeyState(VK_F11) & 0x8000) {
                printf("\nRecalibrating - move to ALL extremes for 4 seconds...\n");
                axY = {}; axZ = {};
                DWORD t1 = GetTickCount();
                while (GetTickCount() - t1 < 4000) {
                    double rx, ry, rz;
                    if (dhdGetPosition(&rx, &ry, &rz) >= 0) { axY.UpdateReach(ry); axZ.UpdateReach(rz); }
                    Sleep(1);
                }
                double rawHalfY = (axY.absMax - axY.absMin) * 0.5;
                double rawHalfZ = (axZ.absMax - axZ.absMin) * 0.5;
                double fitY = rawHalfY * CALIB_FILL, fitZ = rawHalfZ * CALIB_FILL;
                if (fitY >= CALIB_MIN && fitY <= CALIB_MAX) axY.halfRange = fitY;  // guard against a partial sweep
                if (fitZ >= CALIB_MIN && fitZ <= CALIB_MAX) axZ.halfRange = fitZ;
                double recommend = (axY.halfRange < axZ.halfRange) ? axY.halfRange : axZ.halfRange;
                printf("Measured half-extent:  Y=%.4f m  Z=%.4f m\n", rawHalfY, rawHalfZ);
                printf("Work area now:  Y half=%.4f  Z half=%.4f\n", axY.halfRange, axZ.halfRange);
                printf(">> Permanent fit: set HALF_RANGE_MAX = %.4f in %s\n", recommend, g_configPath);
                QueryPerformanceCounter(&qLast);      // don't let the 4s calibration produce a giant dt
                QueryPerformanceCounter(&qDeadline);
                Sleep(200);
            }
        }

        // ── Loop pacing ────────────────────────────────────────────────────
        // UPDATE_RATE_MS == 0: no wait at all — the dhd USB transactions throttle
        // the loop to ~1000Hz naturally (Sleep(0) still yields the timeslice, which
        // is what was costing rate). UPDATE_RATE_MS >= 1: precise QPC-deadline
        // pacing — coarse Sleep(1) while >1.5ms remain, then yield-spin the rest,
        // instead of raw Sleep()'s 1-2ms lottery.
        if (periodTicks > 0) {
            qDeadline.QuadPart += periodTicks;
            LARGE_INTEGER qChk; QueryPerformanceCounter(&qChk);
            if (qDeadline.QuadPart < qChk.QuadPart) qDeadline = qChk;   
            for (;;) {
                QueryPerformanceCounter(&qChk);
                LONGLONG remain = qDeadline.QuadPart - qChk.QuadPart;
                if (remain <= 0) break;
                double remainMs = remain * 1000.0 / (double)qpf.QuadPart;
                if (remainMs > 1.5) Sleep(1);
                else Sleep(0);
            }
        }
    }

    dhdSetForce(0.0, 0.0, 0.0);
    dhdClose();
    SerialClose();
    DeleteCriticalSection(&g_recoilCS);
    timeEndPeriod(1);
    printf("\nDone.\n");
    return 0;
}