// Omega 7 - Mouse emulation joystick with Force
// Force Dimension Omega.7 -> Pico 2 W (CP2102 serial) -> Xbox 360 controller
//
// Ported from Novint Falcon version.
// Key differences from Falcon build:
//   - Opens Omega.7 explicitly via dhdOpenType(DHD_DEVICE_OMEGA331)
//   - Reads gripper gap for trigger emulation (btn2 = half press, btn1+2 = full press)
//   - Forces sent via dhdSetForceAndTorqueAndGripperForce() (torques zeroed)
//   - Button 0 on the Falcon maps to Omega.7 physical button 0

#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <dhdc.h>
#include <string.h>

#pragma comment(lib, "winmm.lib")


// ── Serial port ────────────────────────────────────────────────────────────
static const char* SERIAL_PORT = "\\\\.\\COM6";
static const DWORD SERIAL_BAUD = 115200;

// ── Packet constants ───────────────────────────────────────────────────────
static const BYTE PACKET_IN_MAGIC = 0xAA;
static const BYTE PACKET_OUT_MAGIC = 0xBB;

// ── Controller config ──────────────────────────────────────────────────────
static const int  UPDATE_RATE_MS = .25;
static const bool INVERT_X = false;
static const bool INVERT_Y = false;

// ── Gripper / trigger thresholds ──────────────────────────────────────────
// Thresholds are expressed as fractions of the *available travel* below the
// calibrated rest gap, so they are independent of where the user holds the
// device at startup.  Travel = restGap - minGap (measured at boot).
//
//   travel fraction 0.0 = gripper at rest (no buttons)
//   travel fraction 1.0 = gripper fully closed
//
// Button 2 (half press) fires once the user has closed past TRIG_HALF of
// available travel.  Button 1 fires at the crisp "break" point TRIG_BREAK,
// and releases when the gap opens back past TRIG_BREAK + TRIG_HYSTERESIS.
static const double TRIG_HALF = 0.55;  // travel fraction for btn2 (higher = shorter free zone)
static const double TRIG_BREAK = 0.78;  // travel fraction for btn1 (the click)
static const double TRIG_HYSTERESIS = 0.07;  // how far back to open before btn1 releases

// Haptic trigger feel — TWO-STAGE TRIGGER
// The stroke has three phases, all with spring force present (it always
// pushes back toward open like a real trigger):
//
//   1. SLACK (0 → TRIG_HALF): light first-stage take-up, rises 0 → TRIG_SLACK_N.
//      Button 2 fires at the end of slack (TRIG_HALF).
//   2. WALL  (TRIG_HALF → TRIG_BREAK): resistance ramps up steeply to TRIG_WALL_N.
//      This is the "detent" — a firm stop you push against and can rest on.
//   3. BREAK (cross TRIG_BREAK): resistance suddenly DROPS to TRIG_POST_N — the
//      crisp give. Button 1 fires. Spring then keeps returning the trigger.
//
// Hysteresis keeps the wall disengaged until the trigger is released back past
// TRIG_BREAK - TRIG_HYSTERESIS, so resting just past the break doesn't re-arm it.
static const double TRIG_SLACK_N = 1.0;   // spring force at end of slack [N]
static const double TRIG_SLACK_K = 6.0;   // slack curve sharpness (higher = flatter sooner)
static const double TRIG_WALL_N = 3.5;   // peak resistance at the detent wall [N]
static const double TRIG_WALL_EXP = 2.5;   // wall steepness (higher = sharper stop near break)
static const double TRIG_POST_N = 1.5;   // spring force just after the break [N]
// Virtual endstop — a steep wall a little past the break that becomes the stop
// for the trigger motion, so the finger never drives into the physical hard stop.
static const double TRIG_STOP = 0.65;  // travel fraction where the endstop wall begins
static const double TRIG_STOP_N = 3.0;   // endstop wall force [N]
static const double TRIG_STOP_EXP = 8.0;   // endstop steepness (higher = softer entry, harder core)
static const double TRIG_STOP_DAMP_N = 40.0;  // extra damping at full endstop penetration [N·s/m]
static const double TRIG_DAMP_N = 0.5;   // base velocity damping coefficient [N·s/m]
// Button 1 break click
static const double TRIG_BREAK_DROP_N = 1.5;   // opening impulse magnitude at btn1 click [N]
static const double TRIG_BREAK_DECAY = 22.0;  // how fast btn1 click impulse fades [1/s]
// Button 2 half-press click (smaller)
static const double TRIG_HALF_DROP_N = 0.5;   // opening impulse at btn2 click [N]
static const double TRIG_HALF_DECAY = 28.0;  // fades faster than btn1 [1/s]

// ── FAR SIDE — pushing OUTWARD (away from operator) past the middle rest ────
// The gripper now rests near the MIDDLE of its travel. Closing toward the
// operator drives the two-stage trigger above (buttons 1 & 2). Opening past
// the rest point engages this mirrored system, whose spring pushes back toward
// the rest (closing direction). It has a single detent:
//   - Button 3 fires when you press OUT against the detent and hold there
//     (a dwell filter distinguishes "press against" from "push through").
//   - Button 4 fires when you break PAST the detent.
// Buttons 3 and 4 are mutually exclusive: a quick stab through the detent gives
// only button 4; a deliberate press-and-hold against it gives button 3.
//
// All far-side fractions are 0 at the rest point, 1 at fully open.
static const double REST_FRAC = 0.5;   // rest as fraction of full travel (0.5 = middle)
// Gripper range is read from the SDK at startup (no prompts). After the device's
// power-up homing the frame is fixed and drift-free, so the gripper's open/closed
// extents are device constants. dhdGetJointAngleRange() gives the gripper joint
// angle limits (index 6); we convert those to a gap [m] using the live angle->gap
// ratio. These fallbacks are used only if the SDK range read fails.
static const double GRIPPER_GAP_PER_RAD = 0.061;  // ~0.030 m gap at ~0.49 rad open (fallback ratio)
static const double GRIPPER_MAX_RAD_FB = 0.49;   // fallback gripper full-open angle [rad]
static const double FAR_WALL_START = 0.25;  // far travel fraction where the detent wall begins
static const double FAR_BREAK = 0.65;  // far travel fraction where button 4 breaks through
static const double FAR_HYSTERESIS = 0.07;  // release margin for button 4
static const DWORD  FAR_DWELL_MS = 100;    // ms held at detent before button 3 registers
static const double FAR_SLACK_N = 1.0;   // outward slack spring level [N]
static const double FAR_SLACK_K = 6.0;   // slack curve sharpness
static const double FAR_WALL_N = 2.5;   // detent wall peak [N]
static const double FAR_WALL_EXP = 2.5;   // detent steepness
static const double FAR_POST_N = 1.2;   // spring force just past the break [N]
static const double FAR_STOP = 0.85;  // far travel fraction where the open-side endstop begins
static const double FAR_STOP_N = 2.0;   // far endstop wall force [N]
static const double FAR_STOP_EXP = 2.0;   // far endstop steepness
static const double FAR_STOP_DAMP_N = 35.0;  // extra damping at full far-endstop penetration [N·s/m]
static const double FAR_BREAK_DROP_N = 1.5;   // button 4 click impulse magnitude [N]
static const double FAR_BREAK_DECAY = 22.0;  // button 4 click impulse fade [1/s]

// ── WRIST PITCH/YAW FINE-TUNE (velocity-based) ─────────────────────────────
// The Omega.7 has an active 3-DOF wrist. Rotating the handle adds to the SAME
// stick axes the end-effector translation drives — and it behaves like the
// translation does: the RATE of wrist rotation drives the output, so the aim
// adjusts while you're actively turning/tilting and STOPS when you hold still.
// Mapping: yaw rate -> stickX (same axis as Y translation), pitch rate -> stickY.
//
// dhdGetOrientationRad returns three wrist angles (oa, ob, og). Which physical
// rotation each represents depends on the device frame, so the indices/signs
// are adjustable. Turn on debug (F10): it prints the raw angles and rates so
// you can see which one changes when you yaw vs pitch, then set IDX/SIGN to match.
static const int    WRIST_FINE_ENABLE = 1;     // 0 = disable wrist fine-tune entirely
static const int    WRIST_YAW_IDX = 2;     // which angle is yaw   (0=oa, 1=ob, 2=og)
static const int    WRIST_PITCH_IDX = 1;     // which angle is pitch (0=oa, 1=ob, 2=og)
static const double WRIST_YAW_SIGN = -1.0;   // flip to -1.0 if yaw nudges the wrong way
static const double WRIST_PITCH_SIGN = 1.0;   // flip to -1.0 if pitch nudges the wrong way
static const double WRIST_VEL_DEADZONE = 0.02;  // rate below this is ignored [rad/s] (kills drift/noise)
static const double WRIST_VEL_SENS = 0.32;  // stick output per rad/s of wrist rate
static const double WRIST_VEL_MAX = 0.40;  // contribution clamp (0..1) — caps fast flicks
static const double WRIST_SMOOTH_HZ = 35.0;  // low-pass on wrist angle before differencing
static const double WRIST_RATE_DT = 0.020; // fixed time base for the rate difference [s] (de-notches)
static const double WRIST_RATE_SMOOTH_HZ = 8.0; // low-pass on the rate itself (smooths the lurches)

// ──Velocity settings ──────────────────────────────────────────────────────
static const double VEL_DEADZONE = 0.0001;
static const double VEL_VLOW_SENS = 22.0;
static const double VEL_VLOW_CURVE = 0.80;
static const double VEL_LOW_SENS = 18.0;
static const double VEL_LOW_CURVE = 0.88;
static const double VEL_HIGH_SENS = 12.0;
static const double VEL_HIGH_CURVE = 0.95;
static const double VEL_BLEND_VLOW = 0.004;
static const double VEL_BLEND_LOW = 0.025;
static const double VEL_BLEND_HIGH = 0.15;
static const double SOFT_RAMP = 0.004;
static const double SHORT_VEL_NOISE = 0.0005;
static const double VEL_VLOW_POS_SCALE = 11.7;
double posBlendMax = VEL_BLEND_LOW * 2.0;

static const double PUSH_ENTER_RAD = 0.56;
static const double PUSH_EXIT_RAD = 0.56;
static const double PUSH_SPEED_BASE = 0.44;
static const double PUSH_SPEED_MAX = 8.0;
static const double HALF_RANGE_MAX = 0.10;

static const double PUSH_DAMP_COEFF = 0.9;
static const double PUSH_DAMP_DECAY = 3.0;

static const double FRICTION_CANCEL = 0.2;
static const double FRICTION_VEL_MIN = 0.0002;
static const double FRICTION_VEL_MAX = 0.02;

static const double FORCE_SPRING_START = 0.3;
static const double FORCE_MAX_RAD = 0.88;
static const double FORCE_MAX_N = 3.0;
static const double FORCE_DAMPING = 3.0;
static const double FORCE_EXPONENT = 2.3;

static const double GRAVITY_COMP_SCALE = 1.0;  // Omega.7 gravity comp is well-calibrated; keep at 1.0

// ── Force model (Omega.7): signal-driven push + ambient rumble ─────────────
// Both effects are driven by the LIVE incoming rumble signal (g_rumble*Peak),
// so amplitude sets per-frame strength and DURATION accumulates: a stronger or
// longer signal builds and sustains more force. Trigger state selects which:
//   - Trigger HELD  -> directional push: BACK toward operator (+X) and UP.
//   - No trigger     -> ambient random Y/Z rumble (atmosphere/impacts).
//
// The incoming signal is combined and compressed into a 0..1 "drive":
//   drive = clamp( (L*LARGE + S*SMALL) ^ CURVE / MAG_SCALE , 0..1 )
static const double RUMBLE_LARGE_SCALE = 1.0;   // weight of the "large" rumble byte
static const double RUMBLE_SMALL_SCALE = 0.5;   // weight of the "small" rumble byte
static const double RECOIL_CURVE = 0.50;  // <1 boosts weak signals; 1.0 = linear
static const double RECOIL_MAG_SCALE = 1.0;   // divisor so a full-scale signal maps near drive=1
static const double RUMBLE_SIGNAL_DECAY_HZ = 25.0; // how fast the live signal fades when packets stop [1/s]
static const DWORD  RECOIL_WINDOW_MS = 250;   // keep "trigger active" this long after btn1 release

// ── Directional push (trigger held) ───────────────────────────────────────
// Each frame the signal drive injects force into an accumulator that decays at
// PUSH_DECAY_HZ. Brief signal -> a punch that decays away; sustained signal ->
// the injection outpaces the decay and pressure builds and holds. Strong signal
// -> bigger injection per frame -> heavier push. This gives per-weapon feel.
static const double PUSH_INJECT_N = 90.0;  // push force injected per second at full drive [N/s]
static const double PUSH_BACK_N = 1.0;   // back (+X) direction weight
static const double PUSH_UP_N = 0.5;   // up direction weight (relative to back)
static const double PUSH_UP_AXIS_IS_Z = 1.0;   // 1.0 = "up" is +Z (forceZ); 0.0 = +Y
static const double PUSH_DECAY_HZ = 9.5;   // accumulator decay [1/s] — discrete<->sustained crossover
static const double PUSH_MAX_N = 2.0;   // clamp on accumulated push magnitude [N]
static const double PUSH_MIN_N = 0.05;  // below this the push is considered finished

// ── Recoil aim compensation ────────────────────────────────────────────────
// The push physically shoves the handle (and its rebound when it fades), which
// the velocity estimator would read as aiming motion and knock the cursor off
// target. We project the aim-plane velocity (Y,Z) onto the current recoil
// direction and attenuate that PARALLEL component, leaving perpendicular motion
// (your real aim corrections) intact. Strength scales with how hard it's kicking.
static const double RECOIL_COMP_PARALLEL = 0.90;  // fraction of along-kick velocity removed at full push (0..1)
static const double RECOIL_COMP_PERP = 0.25;  // fraction of cross-kick velocity removed at full push
static const double RECOIL_COMP_HOLD_MS = 90;    // keep compensating this long after push fades [ms] (settle)

// ── Ambient rumble (no trigger) ────────────────────────────────────────────
// Random-direction Y/Z buzz whose magnitude scales with the live signal.
// Direction re-randomizes every AMBIENT_DIR_MS for a textured feel.
static const double AMBIENT_SCALE_N = 3.0;   // ambient force at full drive [N]
static const DWORD  AMBIENT_DIR_MS = 40;    // ms between new random direction targets
static const double AMBIENT_MIN_DRIVE = 0.02;  // below this drive, ambient is silent
static const double AMBIENT_SMOOTH_HZ = 7.0;   // magnitude attack/release low-pass [Hz] — lower = softer, more "push"
static const double AMBIENT_DIR_SLEW_HZ = 6.0; // how fast direction eases toward its target [Hz] — lower = flowier

// Aim damping while firing — softens stick during recoil
static const double RECOIL_AIM_DAMP = 0.4;   // stick multiplier floor while firing (0=frozen,1=none)
static const double RECOIL_DAMP_DECAY = 8.0;   // how fast aim-damp envelope fades [1/s]

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
static DWORD           g_btn1Released = 0;
static double          g_pushDamp = 0.0;
static volatile float  g_rumbleLargePeak = 0.0f;  // latest incoming bytes (debug/telemetry)
static volatile float  g_rumbleSmallPeak = 0.0f;
static double          g_recoilDampEnv = 0.0;    // aim-damp envelope while firing
// Discrete-push recoil accumulator (replaces the rumble model)
static double          g_pushForce = 0.0;    // current accumulated push magnitude [N]
// Recoil aim compensation: unit direction of the kick in the aim plane (Y,Z)
// and a hold timer so we keep compensating through the settle after it fades.
static double          g_recoilDirY = 0.0;
static double          g_recoilDirZ = 1.0;
static double          g_recoilCompEnv = 0.0;    // 0..1 compensation strength envelope
static DWORD           g_recoilCompHold = 0;      // tick until which to keep compensating
static volatile bool   g_running = true;
static volatile uint8_t g_btnMask = 0;
static volatile uint8_t g_dhdButtons = 0;

// ── Gripper / trigger state ────────────────────────────────────────────────
static double g_gripRestGap = -1.0;  // calibrated rest gap [m] — now the MIDDLE
static double g_gripMinGap = 0.0;  // measured min (fully closed) gap [m]
static double g_gripMaxGap = 0.0;  // measured max (fully open) gap [m]
static double g_opTravel = 0.0;  // restGap - minGap [m]  (operator/closing side)
static double g_farTravel = 0.0;  // maxGap - restGap [m]  (far/opening side)
static bool   g_btn1Latched = false; // true while button 1 held after operator break
static double g_breakImpulse = 0.0;  // decaying click force for btn1 [N]
static double g_halfImpulse = 0.0;  // decaying click force for btn2 [N]
// Far-side state
static bool   g_btn4Latched = false; // true while button 4 held after far break
static bool   g_btn3Active = false; // true while pressing against the far detent
static DWORD  g_farWallEnter = 0;     // tick when the far detent zone was entered (0 = not in it)
static double g_farBreakImpulse = 0.0;   // decaying click force for btn4 [N]
// Wrist fine-tune state (velocity-based)
static bool   g_wristSeeded = false;   // filter/prev-angle seeded yet?
static double g_wristYawF = 0.0;     // low-pass-filtered yaw angle [rad]
static double g_wristPitchF = 0.0;     // low-pass-filtered pitch angle [rad]
static double g_wristYawPrev = 0.0;     // yaw angle at last rate sample [rad]
static double g_wristPitchPrev = 0.0;     // pitch angle at last rate sample [rad]
static double g_wristRateAccum = 0.0;     // time accumulated since last rate sample [s]
static double g_wristYawRateS = 0.0;     // smoothed yaw rate [rad/s]
static double g_wristPitchRateS = 0.0;     // smoothed pitch rate [rad/s]

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
                // Record the latest live signal. Both recoil (trigger held) and
                // ambient rumble (no trigger) read these continuously, so a
                // weak/brief signal and a strong/sustained one feel different.
                g_rumbleLargePeak = inL;
                g_rumbleSmallPeak = inS;
                static DWORD lastRawPrint = 0;
                if (GetTickCount() - lastRawPrint > 100) {
                    lastRawPrint = GetTickCount();
                    printf("\nRAW: L=%.3f S=%.3f\n", inL, inS);
                }
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
    int    zeroCount = 0;
    double slowRef = 0.0;
    bool   slowRefSeeded = false;

    static const int VEL_WINDOW = 6;
    double posHistory[8] = {};
    double timeHistory[8] = {};
    int    posIdx = 0;
    bool   posSeeded = false;

    // Lock the workspace ZERO POINT (estCenter) to a fixed value measured once
    // at startup. The Omega.7 holds center reliably, so we do NOT auto-recenter
    // estCenter from reach. NOTE: this does NOT touch slowRef — slowRef is the
    // slowly-drifting reference that powers the low-speed mouse/position-delta
    // assist, and it must keep drifting for the center zone to feel mouse-like.
    void LockCenter(double center) {
        estCenter = center;
        seeded = true;
        absMin = absMax = center; // kept consistent, but no longer used to recenter
    }

    // slowRef: slow (1.2 Hz) low-pass of position. PosError = pos - slowRef is a
    // high-pass = recent motion relative to where you've been hovering, which is
    // the mouse-like low-speed signal (returns to 0 when you hold still). This is
    // separate from the locked workspace center and MUST keep drifting.
    void UpdateSlowRef(double pos, double dt) {
        if (!slowRefSeeded) { slowRef = pos; slowRefSeeded = true; return; }
        double alpha = 1.0 - exp(-2.0 * 3.14159265 * 1.2 * dt);
        slowRef += alpha * (pos - slowRef);
    }

    double PosError(double pos) const {
        if (halfRange < 0.001) return 0.0;
        double e = (pos - slowRef) / halfRange;   // recent deviation, not absolute offset
        return e < -1.0 ? -1.0 : (e > 1.0 ? 1.0 : e);
    }

    void UpdateVelocity(double pos, double dt) {
        if (!posSeeded) {
            for (int i = 0; i < VEL_WINDOW; i++) {
                posHistory[i] = pos;
                timeHistory[i] = dt > 0.0 ? dt : 0.001;
            }
            posSeeded = true;
        }
        posHistory[posIdx] = pos;
        timeHistory[posIdx] = dt > 0.0 ? dt : 0.001;
        posIdx = (posIdx + 1) % VEL_WINDOW;

        double oldPos = posHistory[posIdx];
        double elapsed = 0.0;
        for (int i = 0; i < VEL_WINDOW; i++) elapsed += timeHistory[i];
        double avgVel = (elapsed > 0.0001) ? (pos - oldPos) / elapsed : 0.0;

        int    prev2 = (posIdx + VEL_WINDOW - 2) % VEL_WINDOW;
        double t2 = timeHistory[(posIdx + VEL_WINDOW - 1) % VEL_WINDOW]
            + timeHistory[(posIdx + VEL_WINDOW - 2) % VEL_WINDOW];
        double shortVel = (t2 > 0.0001) ? (pos - posHistory[prev2]) / t2 : 0.0;

        if (fabs(shortVel) > SHORT_VEL_NOISE && fabs(shortVel) > fabs(avgVel))
            rawVel = shortVel;
        else
            rawVel = avgVel;

        double velSpeed = fabs(rawVel);
        double fc;
        if (velSpeed < VEL_BLEND_VLOW) {
            fc = 9.0;
        }
        else if (velSpeed < VEL_BLEND_LOW) {
            double t = (velSpeed - VEL_BLEND_VLOW) / (VEL_BLEND_LOW - VEL_BLEND_VLOW);
            t = t * t * (3.0 - 2.0 * t);
            fc = 9.0 + (18.0 - 9.0) * t;
        }
        else if (velSpeed < VEL_BLEND_HIGH) {
            double t = (velSpeed - VEL_BLEND_LOW) / (VEL_BLEND_HIGH - VEL_BLEND_LOW);
            t = t * t * (3.0 - 2.0 * t);
            fc = 18.0 + (30.0 - 18.0) * t;
        }
        else {
            double t = (velSpeed - VEL_BLEND_HIGH) / (HALF_RANGE_MAX - VEL_BLEND_HIGH);
            t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
            fc = 30.0 + (55.0 - 30.0) * t;
        }
        double alpha = 1.0 - exp(-2.0 * 3.14159265 * fc * dt);
        smoothVel += alpha * (rawVel - smoothVel);

        if (fabs(rawVel) < VEL_DEADZONE && fabs(smoothVel) < VEL_DEADZONE * 2.0) {
            zeroCount++;
            if (zeroCount > 10) smoothVel = 0.0;
        }
        else {
            zeroCount = 0;
        }
        lastPos = pos;
    }

    double Offset(double pos) const {
        if (halfRange < 0.001) return 0.0;
        double o = (pos - estCenter) / halfRange;
        return o < -1.0 ? -1.0 : (o > 1.0 ? 1.0 : o);
    }
};

// ── Push zone ─────────────────────────────────────────────────────────────
struct PushState2D {
    bool   inPushY = false;
    bool   inPushZ = false;
    double entryVelY = 0.0;
    double entryVelZ = 0.0;

    bool Update(double offY, double offZ, double velY, double velZ,
        double& outX, double& outY)
    {
        if (inPushY) {
            if (fabs(offY) < PUSH_EXIT_RAD) { inPushY = false; entryVelY = 0.0; }
        }
        else {
            if (fabs(offY) >= PUSH_ENTER_RAD) { inPushY = true; entryVelY = outX; }
        }
        if (inPushZ) {
            if (fabs(offZ) < PUSH_EXIT_RAD) { inPushZ = false; entryVelZ = 0.0; }
        }
        else {
            if (fabs(offZ) >= PUSH_ENTER_RAD) { inPushZ = true; entryVelZ = outY; }
        }

        bool inPush = inPushY || inPushZ;
        outX = outY = 0.0;

        if (inPushY) {
            double excess = (fabs(offY) - PUSH_ENTER_RAD) / (1.0 - PUSH_ENTER_RAD);
            excess = excess < 0.0 ? 0.0 : (excess > 1.0 ? 1.0 : excess);
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
            outY = sign * mag;
        }
        return inPush;
    }
};

static int16_t ToStick(double v) {
    if (v > 1.0) v = 1.0;
    if (v < -1.0) v = -1.0;
    return (int16_t)(v * 32767.0);
}

// ── Force output (Omega.7 version) ─────────────────────────────────────────
// The Omega.7 needs dhdSetForceAndTorqueAndGripperForce().
// Torques are zeroed; wrist control is not used here.
static void ApplyForces(double y, double z,
    const AxisState& axY, const AxisState& axZ,
    double velY, double velZ,
    double rumX, double rumY, double rumZ,
    double gripForce)
{
    double offY = axY.Offset(y), offZ = axZ.Offset(z);
    double r = sqrt(offY * offY + offZ * offZ);
    double forceY = 0.0, forceZ = 0.0;

    if (r > FORCE_SPRING_START) {
        double dirY = offY / r, dirZ = offZ / r;
        double t = (r - FORCE_SPRING_START) / (FORCE_MAX_RAD - FORCE_SPRING_START);
        t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
        double mag = pow(t, FORCE_EXPONENT) * FORCE_MAX_N;
        double vOut = velY * dirY + velZ * dirZ;
        if (vOut > 0.0) {
            double proximity = (r - FORCE_SPRING_START) / (PUSH_ENTER_RAD - FORCE_SPRING_START);
            proximity = proximity < 0.0 ? 0.0 : (proximity > 1.0 ? 1.0 : proximity);
            static const double DAMP_VEL_MIN = 0.010;
            static const double DAMP_VEL_MAX = 0.020;
            double velMag = sqrt(velY * velY + velZ * velZ);
            double velGate = (velMag - DAMP_VEL_MIN) / (DAMP_VEL_MAX - DAMP_VEL_MIN);
            velGate = velGate < 0.0 ? 0.0 : (velGate > 1.0 ? 1.0 : velGate);
            velGate = velGate * velGate * (3.0 - 2.0 * velGate);
            double dynamicDamp = FORCE_DAMPING * (1.0 + proximity * 2.0 * velGate);
            mag += vOut * dynamicDamp;
        }
        forceY = -dirY * mag;
        forceZ = -dirZ * mag;
    }

    forceY += rumY;
    forceZ += rumZ;

    // Friction cancellation
    static const double FRIC_VEL_MIN = 0.003;
    static const double FRIC_VEL_MAX = 0.08;
    double velMag = sqrt(velY * velY + velZ * velZ);
    if (velMag > FRIC_VEL_MIN) {
        double fadeIn = fmin((velMag - FRIC_VEL_MIN) / (FRIC_VEL_MIN * 3.0), 1.0);
        double fadeOut = fmax(1.0 - (velMag - FRIC_VEL_MIN) / (FRIC_VEL_MAX - FRIC_VEL_MIN), 0.0);
        double blend = fadeIn * fadeOut;
        forceY += (velY / velMag) * FRICTION_CANCEL * blend;
        forceZ += (velZ / velMag) * FRICTION_CANCEL * blend;
    }

    double yzMag = sqrt(forceY * forceY + forceZ * forceZ);
    if (yzMag > 7.8) { double s = 7.8 / yzMag; forceY *= s; forceZ *= s; }
    if (rumX > FORCE_MAX_N) rumX = FORCE_MAX_N;
    if (rumX < -FORCE_MAX_N) rumX = -FORCE_MAX_N;

    // Omega.7: force on X, Y (= Falcon Y,Z axes), torques zeroed, gripper force
    dhdSetForceAndTorqueAndGripperForce(
        rumX, forceY, forceZ,   // Fx Fy Fz
        0.0, 0.0, 0.0,       // Tx Ty Tz  (wrist torques, unused)
        gripForce                // gripper force [N]
    );
}

// ── Main ───────────────────────────────────────────────────────────────────
int main() {
    InitializeCriticalSection(&g_recoilCS);

    printf("FalconJoyForce (Omega.7 port) - Omega.7 -> Pico 2W (Serial) -> Xbox Controller\n\n");

    printf("Opening %s at %d baud...\n", SERIAL_PORT, SERIAL_BAUD);
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

    // ── Open Omega.7 explicitly ────────────────────────────────────────────
    // dhdOpenType() opens the first device of the requested type.
    // DHD_DEVICE_OMEGA331 = right-handed Omega.7.
    // Use DHD_DEVICE_OMEGA331_LEFT for the left-handed variant.
    if (dhdOpenType(DHD_DEVICE_OMEGA331) < 0) {
        printf("ERROR: Cannot open Omega.7 (%s).\n", dhdErrorGetLastStr());
        printf("Is the device connected and powered on?\n");
        SerialClose(); getchar(); return -1;
    }

    // The Omega.7 gripper force is always active and cannot be toggled —
    // dhdEnableGripperForce() returns -1 "operation not available" on this device.
    // dhdSetMaxGripperForce sets the hardware clamp. The default is 1 N which
    // silently caps our spring. Set it to 8 N (motor continuous rating).
    dhdEnableForce(DHD_ON);
    dhdEmulateButton(DHD_OFF);
    dhdSetMaxGripperForce(8.0);
    printf("Gripper force max set to %.1f N  (limit was %.1f N)\n",
        8.0, dhdGetMaxGripperForce());
    printf("dhdHasActiveGripper = %s\n", dhdHasActiveGripper() ? "YES" : "NO");
    timeBeginPeriod(1);

    // Gravity compensation — Omega.7 has excellent built-in gravity comp.
    // Keep GRAVITY_COMP_SCALE at 1.0 unless you have a custom end-effector.
    dhdSetGravityCompensation(DHD_ON);
    dhdSetStandardGravity(9.81 * GRAVITY_COMP_SCALE);

    printf("Omega.7 ready. SDK %s\n\n", dhdGetSDKVersionStr());
    printf("Serial: %s   Push zone: %.0f%%   Force max rad: %.2f\n",
        SERIAL_PORT, PUSH_ENTER_RAD * 100.0, FORCE_MAX_RAD);

    // ── Gripper range from the homed device (no prompts) ───────────────────
    // After power-up homing the gripper's open/closed extents are fixed device
    // constants, so we read them from the SDK instead of asking the user to
    // open/close every launch. dhdGetJointAngleRange() returns the gripper joint
    // angle limits at index 6 (per the DHD axis convention where index 6 is the
    // gripper DOF); we convert angle->gap with the device's live angle/gap ratio.
    auto autoGripperRange = []() {
        dhdEnableExpertMode();
        double jmin[DHD_MAX_DOF] = {}, jmax[DHD_MAX_DOF] = {};
        double aClosed = 0.0, aOpen = GRIPPER_MAX_RAD_FB;
        if (dhdGetJointAngleRange(jmin, jmax) >= 0) {
            // Gripper joint = index 6. Take the smaller magnitude as closed.
            double a0 = jmin[6], a1 = jmax[6];
            aClosed = fabs(a0) < fabs(a1) ? a0 : a1;
            aOpen = fabs(a0) < fabs(a1) ? a1 : a0;
        }
        // angle -> gap conversion ratio, sampled live if the gripper isn't near
        // closed (where the ratio is ill-defined); else use the fallback ratio.
        double gNow = 0.0, aNow = 0.0, k = GRIPPER_GAP_PER_RAD;
        if (dhdGetGripperGap(&gNow) >= 0 && dhdGetGripperAngleRad(&aNow) >= 0
            && fabs(aNow) > 0.05 && gNow > 0.0) {
            k = gNow / aNow;
        }
        dhdDisableExpertMode();
        g_gripMinGap = fabs(k * aClosed);
        g_gripMaxGap = fabs(k * aOpen);
        if (g_gripMaxGap <= g_gripMinGap)             // safety fallback
            g_gripMaxGap = g_gripMinGap + 0.030;
    };
    autoGripperRange();

    // Rest near the middle (REST_FRAC of the full travel up from closed).
    g_gripRestGap = g_gripMinGap + (g_gripMaxGap - g_gripMinGap) * REST_FRAC;
    g_opTravel = g_gripRestGap - g_gripMinGap;   // rest → closed
    g_farTravel = g_gripMaxGap - g_gripRestGap;  // rest → open
    if (g_opTravel < 0.005) g_opTravel = 0.005;
    if (g_farTravel < 0.005) g_farTravel = 0.005;

    printf("Gripper range (SDK, no homing): closed=%.4fm open=%.4fm  rest(mid)=%.4fm\n",
        g_gripMinGap, g_gripMaxGap, g_gripRestGap);
    printf("Operator (close): btn2@%.0f%%  btn1@%.0f%%   Far (open): btn3@detent  btn4@%.0f%%\n",
        TRIG_HALF * 100.0, TRIG_BREAK * 100.0, FAR_BREAK * 100.0);
    printf("F9=quit  F10=debug  F11=recalibrate  F12=gripper sign test\n\n");

    AxisState   axY, axZ;
    PushState2D push;

    // ── Lock the center to the device origin (Omega.7) ─────────────────────
    // After homing, positions are reported relative to the fixed calibrated
    // origin, so the workspace center IS (0,0,0). No neutral-capture prompt.
    axY.LockCenter(0.0);
    axZ.LockCenter(0.0);
    printf("Center locked to device origin (0,0,0).\n");

    DWORD  lastFrame = GetTickCount();
    DWORD  lastStats = GetTickCount();
    int    hz = 0;
    bool   debugMode = false;
    bool   btn1WasHeld = false;
    g_btn1Released = GetTickCount();

    double stickX = 0.0, stickY = 0.0;
    bool   recoilActive = false;
    bool   wasRecoilActive = false;
    double stickScale = 1.0;

    while (true) {
        DWORD  now = GetTickCount();
        double dt = (now - lastFrame) / 1000.0;
        if (dt > 0.05) dt = 0.01;
        lastFrame = now;

        // ── Read position ──────────────────────────────────────────────────
        double x, y, z;
        if (dhdGetPosition(&x, &y, &z) < 0) {
            printf("Lost Omega.7: %s\n", dhdErrorGetLastStr());
            break;
        }

        axY.UpdateVelocity(y, dt);
        axZ.UpdateVelocity(z, dt);
        // estCenter (workspace zero) is locked at startup — no reach recentering.
        // slowRef still drifts: it powers the low-speed mouse/position-delta assist.
        axY.UpdateSlowRef(y, dt);
        axZ.UpdateSlowRef(z, dt);

        // ── Read gripper, compute travel fraction and gripper velocity ────────
        // travel = 0.0  →  gripper at rest
        // travel = 1.0  →  gripper fully closed
        //
        // OBSERVED sign convention:
        //   positive gripper force → motor pushes gripper OPEN  (resists closing)
        //   negative gripper force → motor pushes gripper CLOSED
        //
        // We try dhdGetGripperGap first; if it returns an error we fall back to
        // the angle reading converted to a gap estimate so the fractions are always valid.
        double gripperGap = g_gripRestGap;
        if (dhdGetGripperGap(&gripperGap) < 0) {
            // Fallback: read angle (rad) and scale to approximate gap in metres.
            // Omega.7 max angle ~28 deg = 0.49 rad maps to ~GRIPPER_MAX_GAP.
            double ang = 0.0;
            if (dhdGetGripperAngleRad(&ang) >= 0)
                gripperGap = g_gripRestGap * (ang / 0.49);
        }
        // Sanity-clamp: reject obviously bad readings
        if (gripperGap < 0.0)          gripperGap = 0.0;
        if (gripperGap > g_gripMaxGap * 1.5) gripperGap = g_gripMaxGap;

        // Two signed travel fractions, both 0 at the middle rest:
        //   opFrac  rises 0→1 as the gripper CLOSES toward the operator.
        //   farFrac rises 0→1 as the gripper OPENS away from the operator.
        // Only one is nonzero at a time (the other is clamped to 0).
        bool   operatorSide = (gripperGap <= g_gripRestGap);
        double opFrac = (g_opTravel > 0.0) ? (g_gripRestGap - gripperGap) / g_opTravel : 0.0;
        double farFrac = (g_farTravel > 0.0) ? (gripperGap - g_gripRestGap) / g_farTravel : 0.0;
        if (opFrac < 0.0) opFrac = 0.0;  if (opFrac > 1.0) opFrac = 1.0;
        if (farFrac < 0.0) farFrac = 0.0;  if (farFrac > 1.0) farFrac = 1.0;

        // Gripper velocity (gap/s, positive = closing).
        // Low-pass at 45 Hz — fast enough that the damping term stays in phase
        // with motion. Too-laggy velocity injects energy and worsens wall bounce.
        static double lastGripGap = -1.0;
        static double gripVelSmooth = 0.0;
        if (lastGripGap < 0.0) lastGripGap = gripperGap;
        double rawGripVel = (dt > 0.0) ? (lastGripGap - gripperGap) / dt : 0.0;
        lastGripGap = gripperGap;
        double gvAlpha = 1.0 - exp(-2.0 * 3.14159265 * 45.0 * dt);
        gripVelSmooth += gvAlpha * (rawGripVel - gripVelSmooth);

        // ── Operator-side state machine (close): btn1 break ─────────────────
        static bool wasPastBreak = false;
        bool isPastBreak = (opFrac >= TRIG_BREAK);
        if (isPastBreak && !wasPastBreak) {
            g_breakImpulse = TRIG_BREAK_DROP_N;
            g_btn1Latched = true;
        }
        wasPastBreak = isPastBreak;
        if (g_btn1Latched && opFrac < (TRIG_BREAK - TRIG_HYSTERESIS)) {
            g_btn1Latched = false;
        }

        // ── Far-side state machine (open): btn4 break + btn3 detent ─────────
        static bool wasPastFarBreak = false;
        bool isPastFarBreak = (farFrac >= FAR_BREAK);
        if (isPastFarBreak && !wasPastFarBreak) {
            g_farBreakImpulse = FAR_BREAK_DROP_N;
            g_btn4Latched = true;        // broke past the detent
        }
        wasPastFarBreak = isPastFarBreak;
        if (g_btn4Latched && farFrac < (FAR_BREAK - FAR_HYSTERESIS)) {
            g_btn4Latched = false;
        }
        // Button 3: pressing AGAINST the detent (in the wall zone, not broken).
        // A dwell filter separates "press and hold against it" (btn3) from a
        // quick "stab through" (which skips btn3 and only fires btn4).
        bool inFarWall = (farFrac >= FAR_WALL_START && farFrac < FAR_BREAK);
        if (inFarWall && !g_btn4Latched) {
            if (g_farWallEnter == 0) g_farWallEnter = now;
            g_btn3Active = (now - g_farWallEnter >= FAR_DWELL_MS);
        }
        else {
            g_farWallEnter = 0;
            g_btn3Active = false;
        }
        if (g_btn4Latched) g_btn3Active = false;   // mutually exclusive with btn4

        // ── Haptic trigger force ────────────────────────────────────────────
        // Operator side: positive force (pushes OPEN, back toward rest).
        // Far side:      negative force (pushes CLOSED, back toward rest).
        // Both sides are a slack→detent→break→post→endstop profile; the far
        // side has a single detent (the btn3 wall) and breaks at btn4.
        double springSigned = 0.0;     // signed force before damping
        double stopPenetration = 0.0;  // 0 outside an endstop, 0..1 inside it

        if (operatorSide) {
            double springF;
            if (!g_btn1Latched) {
                if (opFrac < TRIG_HALF) {
                    double s = opFrac / TRIG_HALF;
                    springF = TRIG_SLACK_N * (1.0 - exp(-TRIG_SLACK_K * s));
                }
                else {
                    double w = (opFrac - TRIG_HALF) / (TRIG_BREAK - TRIG_HALF);
                    if (w > 1.0) w = 1.0;
                    springF = TRIG_SLACK_N + (TRIG_WALL_N - TRIG_SLACK_N) * pow(w, TRIG_WALL_EXP);
                }
            }
            else {
                if (opFrac < TRIG_STOP) {
                    double p = (opFrac - TRIG_BREAK) / (TRIG_STOP - TRIG_BREAK);
                    p = p < 0.0 ? 0.0 : (p > 1.0 ? 1.0 : p);
                    springF = TRIG_POST_N * (1.0 + 0.3 * p);
                }
                else {
                    double e = (opFrac - TRIG_STOP) / (1.0 - TRIG_STOP);
                    e = e < 0.0 ? 0.0 : (e > 1.0 ? 1.0 : e);
                    stopPenetration = e;
                    double base = TRIG_POST_N * 1.3;
                    springF = base + (TRIG_STOP_N - base) * pow(e, TRIG_STOP_EXP);
                }
            }
            springSigned = +springF;   // opening = pushes back toward rest
        }
        else {
            // FAR SIDE — mirror with a single detent. Magnitude positive here,
            // negated below so it pushes the gripper closed (back to the middle).
            double farF;
            if (!g_btn4Latched) {
                if (farFrac < FAR_WALL_START) {
                    double s = farFrac / FAR_WALL_START;
                    farF = FAR_SLACK_N * (1.0 - exp(-FAR_SLACK_K * s));
                }
                else {
                    double w = (farFrac - FAR_WALL_START) / (FAR_BREAK - FAR_WALL_START);
                    if (w > 1.0) w = 1.0;
                    farF = FAR_SLACK_N + (FAR_WALL_N - FAR_SLACK_N) * pow(w, FAR_WALL_EXP);
                }
            }
            else {
                if (farFrac < FAR_STOP) {
                    double p = (farFrac - FAR_BREAK) / (FAR_STOP - FAR_BREAK);
                    p = p < 0.0 ? 0.0 : (p > 1.0 ? 1.0 : p);
                    farF = FAR_POST_N * (1.0 + 0.3 * p);
                }
                else {
                    double e = (farFrac - FAR_STOP) / (1.0 - FAR_STOP);
                    e = e < 0.0 ? 0.0 : (e > 1.0 ? 1.0 : e);
                    stopPenetration = e;
                    double base = FAR_POST_N * 1.3;
                    farF = base + (FAR_STOP_N - base) * pow(e, FAR_STOP_EXP);
                }
            }
            springSigned = -farF;      // closing = pushes back toward rest
        }

        // Velocity damping. Base is light; heavy penetration-scaled damping is
        // added inside whichever endstop is active so impacts are absorbed, not
        // sprung back (Kelvin–Voigt wall).
        double endstopDamp = operatorSide ? TRIG_STOP_DAMP_N : FAR_STOP_DAMP_N;
        double dampCoeff = TRIG_DAMP_N + endstopDamp * stopPenetration;
        double dampF = dampCoeff * gripVelSmooth;

        double gripForce = springSigned + dampF;
        // Don't let damping reverse the spring's direction (prevents the wall
        // from "grabbing"/sucking past the rest). Clamp net to the spring side.
        if (springSigned >= 0.0) { if (gripForce < 0.0) gripForce = 0.0; }
        else { if (gripForce > 0.0) gripForce = 0.0; }

        // Diagnostic: print gripper state every second. Remove once tuned.
        static DWORD lastGripPrint = 0;
        if (now - lastGripPrint >= 1000) {
            lastGripPrint = now;
            printf("\nGRIP gap=%.4fm  op=%.0f%%%s  far=%.0f%%%s%s  grip=%+.2f\n",
                gripperGap,
                opFrac * 100.0, g_btn1Latched ? "(B1)" : "",
                farFrac * 100.0, g_btn3Active ? "(B3)" : "", g_btn4Latched ? "(B4)" : "",
                gripForce);
        }

        // ── btn2 half-press click (rising edge at operator TRIG_HALF) ──────
        static bool wasPastHalf = false;
        bool isPastHalf = (opFrac >= TRIG_HALF);
        if (isPastHalf && !wasPastHalf) {
            g_halfImpulse = TRIG_HALF_DROP_N;
        }
        if (!isPastHalf) g_halfImpulse = 0.0;  // reset when released back past half
        wasPastHalf = isPastHalf;

        g_halfImpulse -= g_halfImpulse * TRIG_HALF_DECAY * dt;
        if (g_halfImpulse < 0.001) g_halfImpulse = 0.0;
        gripForce += g_halfImpulse;  // opening burst at btn2 threshold

        // ── btn1 break click decay (operator) — opening burst ──────────────
        g_breakImpulse -= g_breakImpulse * TRIG_BREAK_DECAY * dt;
        if (g_breakImpulse < 0.001) g_breakImpulse = 0.0;
        gripForce += g_breakImpulse;  // positive = opening burst at btn1 break

        // ── btn4 break click decay (far) — closing burst ───────────────────
        g_farBreakImpulse -= g_farBreakImpulse * FAR_BREAK_DECAY * dt;
        if (g_farBreakImpulse < 0.001) g_farBreakImpulse = 0.0;
        gripForce -= g_farBreakImpulse;  // negative = closing burst at btn4 break

        // ── Button mask ────────────────────────────────────────────────────
        // Bit layout sent to Pico:
        //   bit 0 = button 1  (operator full press / break)
        //   bit 1 = button 2  (operator half press)
        //   bit 2 = button 3  (far detent press-and-hold)
        //   bit 3 = button 4  (far break-through)
        //   bit 4 = Omega.7 physical button 0
        //   bit 5 = Omega.7 physical button 1
        uint8_t btnMask = 0;

        bool triggerHalf = (opFrac >= TRIG_HALF);  // operator half (btn2)
        bool triggerFull = g_btn1Latched;          // operator break (btn1)

        if (triggerFull)    btnMask |= (1u << 0);  // button 1
        if (triggerHalf)    btnMask |= (1u << 1);  // button 2
        if (g_btn3Active)   btnMask |= (1u << 2);  // button 3
        if (g_btn4Latched)  btnMask |= (1u << 3);  // button 4

        if (dhdGetButton(0) > 0) btnMask |= (1u << 4);  // physical btn 0
        if (dhdGetButton(1) > 0) btnMask |= (1u << 5);  // physical btn 1

        // ── Recoil active window driven by bit 0 (full trigger / btn1) ────
        bool btn1held = (btnMask & (1u << 0)) != 0;
        if (btn1WasHeld && !btn1held) g_btn1Released = now;
        btn1WasHeld = btn1held;
        recoilActive = btn1held || (now - g_btn1Released < RECOIL_WINDOW_MS);

        if (recoilActive && !wasRecoilActive) g_recoilDampEnv = 1.0;
        wasRecoilActive = recoilActive;

        g_recoilDampEnv -= g_recoilDampEnv * RECOIL_DAMP_DECAY * dt;
        if (g_recoilDampEnv < 0.0) g_recoilDampEnv = 0.0;

        // ── Push zone ──────────────────────────────────────────────────────
        double offY = axY.Offset(y), offZ = axZ.Offset(z);
        double stickX = 0.0, stickY = 0.0;
        bool   inPush = push.Update(offY, offZ, axY.smoothVel, axZ.smoothVel, stickX, stickY);

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

        // ── Recoil aim compensation (directional) ──────────────────────────
        // While a push is active (and briefly after, to cover the settle/rebound)
        // the handle physically moves along the recoil direction. Decompose the
        // aim-plane velocity (Y,Z) into a component ALONG the kick and one ACROSS
        // it, attenuate the along-kick component hard and the across-kick lightly,
        // then recombine. This kills the cursor walk from recoil while leaving
        // your real perpendicular aim corrections almost untouched.
        {
            // Current kick direction in the aim plane. Back/+X is not an aim axis,
            // so the felt aim-plane kick is the UP component: +Z (or +Y).
            double ky, kz;
            if (PUSH_UP_AXIS_IS_Z >= 0.5) { ky = 0.0; kz = 1.0; }
            else { ky = 1.0; kz = 0.0; }
            g_recoilDirY = ky;
            g_recoilDirZ = kz;

            // Build the compensation envelope from current push strength, and
            // refresh a short hold window so we keep compensating through the
            // settle after the push itself has decayed away.
            double pushEnv = g_pushForce / PUSH_MAX_N;
            if (pushEnv > 1.0) pushEnv = 1.0;
            if (pushEnv < 0.0) pushEnv = 0.0;
            if (g_pushForce > 0.0) {
                g_recoilCompEnv = pushEnv;
                g_recoilCompHold = now + (DWORD)RECOIL_COMP_HOLD_MS;
            }
            else if (now < g_recoilCompHold) {
                // In the hold window: fade the envelope out smoothly.
                g_recoilCompEnv -= g_recoilCompEnv * 12.0 * dt;
                if (g_recoilCompEnv < 0.0) g_recoilCompEnv = 0.0;
            }
            else {
                g_recoilCompEnv = 0.0;
            }

            if (g_recoilCompEnv > 0.0) {
                double env = g_recoilCompEnv;
                // Project velocity onto kick dir (parallel) and the remainder (perp).
                double vPar = axY.smoothVel * ky + axZ.smoothVel * kz;
                double parY = vPar * ky, parZ = vPar * kz;
                double perpY = axY.smoothVel - parY;
                double perpZ = axZ.smoothVel - parZ;
                // Attenuate each component by its scaled fraction.
                double parScale = 1.0 - RECOIL_COMP_PARALLEL * env;
                double perpScale = 1.0 - RECOIL_COMP_PERP * env;
                axY.smoothVel = parY * parScale + perpY * perpScale;
                axZ.smoothVel = parZ * parScale + perpZ * perpScale;
            }
        }

        // ── Velocity curve → stick output ──────────────────────────────────
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
                double hi = speed * VEL_HIGH_SENS;
                if (hi > 1.0) hi = 1.0;

                double tVlow = (speed - VEL_BLEND_VLOW) / (VEL_BLEND_LOW - VEL_BLEND_VLOW);
                tVlow = tVlow < 0.0 ? 0.0 : (tVlow > 1.0 ? 1.0 : tVlow);
                tVlow = tVlow * tVlow * (3.0 - 2.0 * tVlow);
                double lowBlended = vlow + (lo - vlow) * tVlow;

                double tHi = (speed - VEL_BLEND_LOW) / (VEL_BLEND_HIGH - VEL_BLEND_LOW);
                tHi = tHi < 0.0 ? 0.0 : (tHi > 1.0 ? 1.0 : tHi);
                tHi = tHi * tHi * (3.0 - 2.0 * tHi);

                return sign * (lowBlended * (1.0 - tHi) + hi * tHi) * ramp;
            };

            stickX = curve(axY.smoothVel);
            stickY = curve(axZ.smoothVel);

            double velMag = sqrt(axY.smoothVel * axY.smoothVel + axZ.smoothVel * axZ.smoothVel);
            double posBlend = 1.0 - velMag / posBlendMax;
            posBlend = posBlend < 0.0 ? 0.0 : (posBlend > 1.0 ? 1.0 : posBlend);
            posBlend = posBlend * posBlend * (3.0 - 2.0 * posBlend);

            if (posBlend > 0.0) {
                double pcX = axY.PosError(y) * VEL_VLOW_POS_SCALE;
                double pcY = axZ.PosError(z) * VEL_VLOW_POS_SCALE;
                pcX = pcX < -1.0 ? -1.0 : (pcX > 1.0 ? 1.0 : pcX);
                pcY = pcY < -1.0 ? -1.0 : (pcY > 1.0 ? 1.0 : pcY);
                stickX = stickX * (1.0 - posBlend) + pcX * posBlend;
                stickY = stickY * (1.0 - posBlend) + pcY * posBlend;
            }
        }

        // ── Live signal drive ──────────────────────────────────────────────
        // Read the latest rumble signal, combine + compress into a 0..1 drive.
        // Decay the stored peaks each frame so a one-shot signal fades instead
        // of sticking on; a sustained stream keeps refreshing them near 1.
        double rawMag = (g_rumbleLargePeak * RUMBLE_LARGE_SCALE
            + g_rumbleSmallPeak * RUMBLE_SMALL_SCALE);
        double drive = pow(rawMag, RECOIL_CURVE) / RECOIL_MAG_SCALE;
        if (drive < 0.0) drive = 0.0;  if (drive > 1.0) drive = 1.0;
        // fade the live peaks (only the main loop decays them; serial thread refreshes)
        double sigFade = exp(-RUMBLE_SIGNAL_DECAY_HZ * dt);
        g_rumbleLargePeak = (float)(g_rumbleLargePeak * sigFade);
        g_rumbleSmallPeak = (float)(g_rumbleSmallPeak * sigFade);

        double rumX = 0.0, rumY = 0.0, rumZ = 0.0;

        if (recoilActive) {
            // ── Trigger held: directional push (back + up) ─────────────────
            // Inject force proportional to drive every frame: amplitude sets the
            // injection rate, duration accumulates it. Decay gives the discrete
            // punch / sustained-pressure crossover from a single time constant.
            g_pushForce += drive * PUSH_INJECT_N * dt;
            g_pushForce *= exp(-PUSH_DECAY_HZ * dt);
            if (g_pushForce > PUSH_MAX_N) g_pushForce = PUSH_MAX_N;
            if (g_pushForce < PUSH_MIN_N) g_pushForce = 0.0;

            if (g_pushForce > 0.0) {
                double upFrac = (PUSH_BACK_N > 0.0) ? (PUSH_UP_N / PUSH_BACK_N) : 0.0;
                rumX = g_pushForce;                        // back toward operator
                double up = g_pushForce * upFrac;          // rising component
                if (PUSH_UP_AXIS_IS_Z >= 0.5) rumZ = up;   // up = +Z
                else                          rumY = up;   // up = +Y
            }
        }
        else {
            // ── No trigger: ambient Y/Z rumble (smoothed into a push) ──────
            // Magnitude is attack/release low-passed so it swells in and eases
            // out instead of snapping (less gunshot, more push). Direction picks
            // a new random TARGET periodically and eases toward it rather than
            // jumping, so the force flows around instead of buzzing.
            g_pushForce *= exp(-PUSH_DECAY_HZ * dt);
            if (g_pushForce < PUSH_MIN_N) g_pushForce = 0.0;

            static double ambDirY = 1.0, ambDirZ = 0.0;        // current direction
            static double ambTgtY = 1.0, ambTgtZ = 0.0;        // target direction
            static double ambMag = 0.0;                       // smoothed magnitude [N]
            static DWORD  ambLastDir = 0;

            // New random target direction every AMBIENT_DIR_MS (only while active).
            double target = (drive > AMBIENT_MIN_DRIVE) ? (drive * AMBIENT_SCALE_N) : 0.0;
            if (drive > AMBIENT_MIN_DRIVE && now - ambLastDir > AMBIENT_DIR_MS) {
                ambLastDir = now;
                double angle = ((rand() % 1000) / 1000.0) * 2.0 * 3.14159265;
                ambTgtY = cos(angle);
                ambTgtZ = sin(angle);
            }

            // Ease direction toward the target, then renormalize.
            double dirA = 1.0 - exp(-2.0 * 3.14159265 * AMBIENT_DIR_SLEW_HZ * dt);
            ambDirY += dirA * (ambTgtY - ambDirY);
            ambDirZ += dirA * (ambTgtZ - ambDirZ);
            double dn = sqrt(ambDirY * ambDirY + ambDirZ * ambDirZ);
            if (dn > 1e-6) { ambDirY /= dn; ambDirZ /= dn; }

            // Attack/release the magnitude toward the target level.
            double magA = 1.0 - exp(-2.0 * 3.14159265 * AMBIENT_SMOOTH_HZ * dt);
            ambMag += magA * (target - ambMag);
            if (ambMag < 0.001) ambMag = 0.0;

            rumY = ambDirY * ambMag;
            rumZ = ambDirZ * ambMag;
        }

        ApplyForces(y, z, axY, axZ, axY.smoothVel, axZ.smoothVel,
            rumX, rumY, rumZ, gripForce);

        // ── Wrist pitch/yaw fine-tune (velocity-based) ─────────────────────
        // Drives the stick from the RATE of wrist rotation, like the translation
        // axis: it adjusts while you actively turn/tilt the handle and stops when
        // you hold still. No neutral needed — a static tilt produces zero rate.
        double wristYawRate = 0.0, wristPitchRate = 0.0;
        if (WRIST_FINE_ENABLE) {
            double oa = 0.0, ob = 0.0, og = 0.0;
            if (dhdGetOrientationRad(&oa, &ob, &og) >= 0) {
                double ang[3] = { oa, ob, og };
                double yawRaw = ang[WRIST_YAW_IDX];
                double pitchRaw = ang[WRIST_PITCH_IDX];

                // Low-pass the raw angles first (kills encoder jitter).
                double a = 1.0 - exp(-2.0 * 3.14159265 * WRIST_SMOOTH_HZ * dt);
                if (!g_wristSeeded) {
                    g_wristYawF = yawRaw;   g_wristPitchF = pitchRaw;
                    g_wristYawPrev = yawRaw; g_wristPitchPrev = pitchRaw;
                    g_wristRateAccum = 0.0;
                    g_wristYawRateS = 0.0; g_wristPitchRateS = 0.0;
                    g_wristSeeded = true;
                }
                else {
                    g_wristYawF += a * (yawRaw - g_wristYawF);
                    g_wristPitchF += a * (pitchRaw - g_wristPitchF);
                }

                // Compute the rate over a FIXED time base (WRIST_RATE_DT), not
                // per-frame. At ~1 kHz a single encoder step over a 1 ms frame
                // reads as a huge spike with zeros between — that is the notchy
                // lurching. Accumulating ~20 ms before differencing means several
                // encoder counts fall in each window, so the rate is smooth and
                // proportional to how fast you are actually twisting.
                g_wristRateAccum += dt;
                if (g_wristRateAccum >= WRIST_RATE_DT) {
                    double yRate = (g_wristYawF - g_wristYawPrev) / g_wristRateAccum;
                    double pRate = (g_wristPitchF - g_wristPitchPrev) / g_wristRateAccum;
                    // Low-pass the rate itself to remove any residual step-edges.
                    double ra = 1.0 - exp(-2.0 * 3.14159265 * WRIST_RATE_SMOOTH_HZ * g_wristRateAccum);
                    g_wristYawRateS += ra * (yRate - g_wristYawRateS);
                    g_wristPitchRateS += ra * (pRate - g_wristPitchRateS);
                    g_wristYawPrev = g_wristYawF;
                    g_wristPitchPrev = g_wristPitchF;
                    g_wristRateAccum = 0.0;
                }
                // Between fixed-interval samples the smoothed rate simply holds
                // its last value, so the stick output stays steady while twisting.

                wristYawRate = g_wristYawRateS * WRIST_YAW_SIGN;
                wristPitchRate = g_wristPitchRateS * WRIST_PITCH_SIGN;

                // Rate -> stick delta: deadzone, scale, clamp. Output exists only
                // while the wrist is moving; it returns to 0 when motion stops.
                auto fineRate = [](double rate) -> double {
                    double s = rate >= 0.0 ? 1.0 : -1.0;
                    double m = fabs(rate) - WRIST_VEL_DEADZONE;
                    if (m <= 0.0) return 0.0;
                    double out = s * m * WRIST_VEL_SENS;
                    if (out > WRIST_VEL_MAX) out = WRIST_VEL_MAX;
                    if (out < -WRIST_VEL_MAX) out = -WRIST_VEL_MAX;
                    return out;
                };

                stickX += fineRate(wristYawRate);    // yaw rate   -> Y-translation axis
                stickY += fineRate(wristPitchRate);  // pitch rate -> Z-translation axis

                if (stickX > 1.0) stickX = 1.0;  if (stickX < -1.0) stickX = -1.0;
                if (stickY > 1.0) stickY = 1.0;  if (stickY < -1.0) stickY = -1.0;
            }
        }

        // ── Send controller packet ─────────────────────────────────────────
        double stickScale = 1.0 - g_pushDamp;
        if (stickScale < 0.0) stickScale = 0.0;
        if (recoilActive)
            stickScale *= 1.0 - (1.0 - RECOIL_AIM_DAMP) * g_recoilDampEnv;

        int16_t lx = ToStick(stickX * stickScale * (INVERT_X ? -1.0 : 1.0));
        int16_t ly = ToStick(stickY * stickScale * (INVERT_Y ? -1.0 : 1.0));
        SendControllerPacket(lx, ly, btnMask);

        // ── Status display ─────────────────────────────────────────────────
        hz++;
        if (now - lastStats >= 1000) {
            double sigNow = (g_rumbleLargePeak * RUMBLE_LARGE_SCALE
                + g_rumbleSmallPeak * RUMBLE_SMALL_SCALE);
            double r = sqrt(offY * offY + offZ * offZ);
            if (debugMode) {
                printf("\nY: ctr=%+.4f half=%.4f off=%+.2f vel=%+.4f\n",
                    axY.estCenter, axY.halfRange, offY, axY.smoothVel);
                printf("Z: ctr=%+.4f half=%.4f off=%+.2f vel=%+.4f\n",
                    axZ.estCenter, axZ.halfRange, offZ, axZ.smoothVel);
                printf("grip=%.4fm  op=%.0f%%[%s%s]  far=%.0f%%[%s%s]  btn=%X\n",
                    gripperGap,
                    opFrac * 100.0, triggerHalf ? "2" : "-", triggerFull ? "1" : "-",
                    farFrac * 100.0, g_btn3Active ? "3" : "-", g_btn4Latched ? "4" : "-",
                    btnMask);
                printf("r=%.3f  push=%s  damp=%.2f  stk=(%.2f,%.2f)  pushN=%.2f  sig=%.2f %s\n",
                    r, inPush ? "YES" : "no", g_pushDamp, stickX, stickY,
                    g_pushForce, sigNow, recoilActive ? "PUSH" : "ambient");
                {
                    double oa = 0, ob = 0, og = 0;
                    dhdGetOrientationRad(&oa, &ob, &og);
                    printf("WRIST raw: oa=%+.3f ob=%+.3f og=%+.3f  (yaw=idx%d pitch=idx%d)  rate: yaw=%+.3f pitch=%+.3f rad/s\n",
                        oa, ob, og, WRIST_YAW_IDX, WRIST_PITCH_IDX, wristYawRate, wristPitchRate);
                }
            }
            else {
                int col = (int)((offY + 1.0) / 2.0 * 8.0 + 0.5);
                col = col < 0 ? 0 : (col > 8 ? 8 : col);
                printf("\r%4d Hz  r=%.2f %s  op=%.0f%%[%s%s] far=%.0f%%[%s%s]  [",
                    hz, r, inPush ? "PUSH" : "    ",
                    opFrac * 100.0, triggerHalf ? "2" : "-", triggerFull ? "1" : "-",
                    farFrac * 100.0, g_btn3Active ? "3" : "-", g_btn4Latched ? "4" : "-");
                for (int c = 0; c <= 8; c++) printf(c == col ? "O" : ".");
                printf("]  stk=(%.2f,%.2f)  pushN=%.2f  btn=%X   ",
                    stickX, stickY, g_pushForce, btnMask);
                fflush(stdout);
            }
            hz = 0; lastStats = now;
        }

        if (GetAsyncKeyState(VK_F9) & 0x8000) break;
        if (GetAsyncKeyState(VK_F10) & 0x8000) {
            debugMode = !debugMode;
            printf("\nDebug %s\n", debugMode ? "ON" : "OFF");
            Sleep(200);
        }
        if (GetAsyncKeyState(VK_F11) & 0x8000) {
            printf("\nRecalibrating (no prompts): re-reading SDK gripper range, re-locking center...\n");
            axY = {}; axZ = {};
            axY.LockCenter(0.0);   // center = device origin after homing
            axZ.LockCenter(0.0);

            // Re-read gripper joint angle range from the SDK and convert to gap.
            dhdEnableExpertMode();
            double jmin[DHD_MAX_DOF] = {}, jmax[DHD_MAX_DOF] = {};
            double aClosed = 0.0, aOpen = GRIPPER_MAX_RAD_FB;
            if (dhdGetJointAngleRange(jmin, jmax) >= 0) {
                double a0 = jmin[6], a1 = jmax[6];
                aClosed = fabs(a0) < fabs(a1) ? a0 : a1;
                aOpen = fabs(a0) < fabs(a1) ? a1 : a0;
            }
            double gNow = 0.0, aNow = 0.0, k = GRIPPER_GAP_PER_RAD;
            if (dhdGetGripperGap(&gNow) >= 0 && dhdGetGripperAngleRad(&aNow) >= 0
                && fabs(aNow) > 0.05 && gNow > 0.0) {
                k = gNow / aNow;
            }
            dhdDisableExpertMode();
            g_gripMinGap = fabs(k * aClosed);
            g_gripMaxGap = fabs(k * aOpen);
            if (g_gripMaxGap <= g_gripMinGap) g_gripMaxGap = g_gripMinGap + 0.030;
            g_gripRestGap = g_gripMinGap + (g_gripMaxGap - g_gripMinGap) * REST_FRAC;
            g_opTravel = g_gripRestGap - g_gripMinGap;
            g_farTravel = g_gripMaxGap - g_gripRestGap;
            if (g_opTravel < 0.005) g_opTravel = 0.005;
            if (g_farTravel < 0.005) g_farTravel = 0.005;
            g_btn1Latched = false; g_btn4Latched = false; g_btn3Active = false;
            g_farWallEnter = 0;
            g_breakImpulse = 0.0; g_halfImpulse = 0.0; g_farBreakImpulse = 0.0;
            g_wristSeeded = false;   // re-seed wrist rate tracking
            g_wristRateAccum = 0.0; g_wristYawRateS = 0.0; g_wristPitchRateS = 0.0;
            printf("Center=origin  closed=%.4fm open=%.4fm rest(mid)=%.4fm\n",
                g_gripMinGap, g_gripMaxGap, g_gripRestGap);
            Sleep(200);
        }
        // ── F12: gripper force sign test ───────────────────────────────────
        // Hold F12 to apply +3N. With the corrected convention this should
        // push the gripper OPEN (resist your fingers closing it).
        // If it instead snaps shut, the hardware sign is still inverted —
        // flip the sign of all the TRIG_*_N force constants and both impulses.
        if (GetAsyncKeyState(VK_F12) & 0x8000) {
            static DWORD lastSignTest = 0;
            if (now - lastSignTest > 200) {
                lastSignTest = now;
                printf("\nF12 SIGN TEST: applying +3N (should push gripper OPEN / resist closing)\n");
            }
            dhdSetForceAndTorqueAndGripperForce(0, 0, 0, 0, 0, 0, +3.0);
            continue;
        }
        Sleep(UPDATE_RATE_MS);
    }

    dhdSetForceAndTorqueAndGripperForce(0, 0, 0, 0, 0, 0, 0);
    dhdClose();
    SerialClose();
    DeleteCriticalSection(&g_recoilCS);
    timeEndPeriod(1);
    printf("\nDone.\n");
    return 0;
}