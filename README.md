# TI-RSLK MAX Line Detector Robot

<p align="center">
  <img src="https://img.shields.io/badge/Platform-TI--RSLK%20MAX-cc0000?style=for-the-badge" alt="Platform" />
  <img src="https://img.shields.io/badge/MCU-MSP432P401R-0066cc?style=for-the-badge" alt="MCU" />
  <img src="https://img.shields.io/badge/Toolchain-CCS%20%2B%20GCC-22863a?style=for-the-badge" alt="Toolchain" />
  <img src="https://img.shields.io/badge/RTOS-NoRTOS-444444?style=for-the-badge" alt="NoRTOS" />
</p>

A modular embedded robotics project for the **TI-RSLK MAX** built on the **MSP432P401R**.  
The codebase focuses on a clean state-machine structure for:

- line following
- obstacle confirmation and bypass
- bump-based collision handling
- gated line reacquisition
- calibration and verification flows
- optional UART debug output for tuning

This repository is set up for **Code Composer Studio (CCS)** with the GCC toolchain and keeps the low-level motor, bump, line, and timing drivers separate from the higher-level behavior logic.

## Highlights

- Clean top-level controller in `control.c`
- Dedicated `line_follow.c` for normal tracking behavior
- Dedicated `avoid.c` for obstacle handling and bypass logic
- Compile-time feature flags in `robot_config.h`
- Debug and real-system mode selection from one place
- Calibration, verification, and demo flows already wired into `main.c`
- CCS watch-friendly global debug variables across modules

## System Overview

The robot uses these sensor groups and actuators:

- **8-bit reflectance array** for line detection
- **Front bump switches** for hard contact confirmation
- **Center IR analog sensor** for close obstacle confirmation
- **Dual front ultrasonics** for range-based obstacle awareness
- **Dual DC motors** driven through the TI-RSLK MAX motor stage
- **LaunchPad LEDs** for simple onboard indication
- **XDS110 backchannel UART** for optional debug output

## Control Architecture

The main execution path is intentionally simple:

1. `main.c` initializes clock, timing, motors, and selected app flow.
2. `Control_Update()` arbitrates between normal line following and obstacle-handling behavior.
3. `LineFollow_Update()` handles standard tracking, recovery, and ramp-related logic.
4. `Avoid_Update()` handles obstacle confirmation, bump response, bypass, and gated reacquisition.

### Top-Level Control States

Defined in `control.h`:

- `CONTROL_STATE_LINE_FOLLOW`
- `CONTROL_STATE_OBSTACLE_APPROACH_CONFIRM`
- `CONTROL_STATE_EMERGENCY_BUMP`
- `CONTROL_STATE_OBSTACLE_BYPASS`
- `CONTROL_STATE_LINE_REACQUIRE_GATED`
- `CONTROL_STATE_COLLISION`
- `CONTROL_STATE_EMERGENCY`

### Avoidance States

Defined in `avoid.h`:

- `AVOID_STATE_IDLE`
- `AVOID_STATE_OBSTACLE_APPROACH_CONFIRM`
- `AVOID_STATE_EMERGENCY_BUMP`
- `AVOID_STATE_OBSTACLE_BYPASS`
- `AVOID_STATE_LINE_REACQUIRE_GATED`

## Project Structure

```text
Line_Detector_Robot/
+-- main.c              # Boot, timing, app-mode selection, optional UART debug
+-- control.c/.h        # Top-level behavior arbitration
+-- line_follow.c/.h    # Line tracking and line recovery logic
+-- avoid.c/.h          # Obstacle confirmation, bypass, reacquire gating
+-- collision.c/.h      # Bump-zone interpretation
+-- reaction.c/.h       # Immediate collision reaction behavior
+-- emergency.c/.h      # Emergency handling path
+-- calibration.c/.h    # Sensor calibration flow
+-- verify.c/.h         # Front sensor verification path
+-- demo.c/.h           # Integrated demo / staged verification mode
+-- line.c/.h           # Reflectance array interface
+-- bump.c/.h           # Bump switch interface
+-- ultrasonic.c/.h     # Ultrasonic interface
+-- ir.c/.h             # Center IR interface
+-- motor.c/.h          # Motor driver
+-- robot_config.h      # Main compile-time configuration point
+-- robot_pins.h        # Hardware pin mapping
+-- targetConfigs/      # CCS target configuration files
```

## Hardware Pin Map

From `robot_pins.h`:

### Motors

- Left direction: `P5.4`
- Left PWM: `P2.7`
- Left sleep: `P3.7`
- Right direction: `P5.5`
- Right PWM: `P2.6`
- Right sleep: `P3.6`

### Bump Switches

- `P4.0`, `P4.2`, `P4.3`, `P4.5`, `P4.6`, `P4.7`

### Line Sensor Array

- Emitters: `P5.3`, `P9.2`
- Sensor bits: `P7.0` to `P7.7`

### IR Sensor

- Center IR analog input: `P6.1` / `ADC_INPUT_A14`

### Ultrasonics

- Left trigger: `P6.3`
- Left echo: `P9.1`
- Right trigger: `P6.2`
- Right echo: `P9.0`

### Debug UART

- UART module: `EUSCI_A0_BASE`
- TX: `P1.3`
- RX: `P1.2`
- Intended path: **LaunchPad XDS110 backchannel UART**
- Runtime terminal settings: **115200, 8-N-1**

## Build and Run

### CCS

1. Open the project in **Code Composer Studio**.
2. Select the correct target configuration from `targetConfigs/MSP432P401R.ccxml`.
3. Build the project with the GCC toolchain configuration.
4. Flash the MSP432P401R.
5. Power the robot on a safe test surface before enabling motion.

### Runtime Flow

Typical startup flow in normal operation:

1. SysTick is initialized for timing.
2. Motors are initialized and left idle.
3. Calibration runs.
4. The controller enters either:
   - normal control mode, or
   - demo mode, depending on configuration.

## Configuration

All major compile-time configuration lives in:

- `robot_config.h`

### System Mode Selection

One top-level macro selects the main operating style:

```c
#define SYSTEM_MODE_DEBUG 1U
#define SYSTEM_MODE_REAL  2U

#define ROBOT_SYSTEM_MODE SYSTEM_MODE_REAL
```

This mode derives helper flags such as:

- `ENABLE_UART_DEBUG`
- `ENABLE_SENSOR_PRINTS`
- `ENABLE_STATE_PRINTS`
- `ENABLE_CASE_PRINTS`

### App Mode Selection

Useful app-level options already present:

- `ROBOT_CFG_APP_MODE_CALIBRATION`
- `ROBOT_CFG_APP_MODE`
- `ROBOT_MODE_NORMAL`
- `ROBOT_MODE_DEMO`

### Obstacle / Bypass Features

These can be enabled or disabled from `robot_config.h`:

- `ENABLE_OBSTACLE_FEATURE`
- `ENABLE_BOUNDARY_FOLLOW_FEATURE`
- `ENABLE_REACQUIRE_GATING_FEATURE`
- `ENABLE_IMU_HEADING_GATE`
- `ENABLE_RAMP_FEATURE`
- `ENABLE_PI_AI_HINT_FEATURE`

### Important Tunables

Examples of tunables already exposed in `robot_config.h`:

- line-follow duty and PD tuning
- obstacle warn and stop thresholds
- IR distance thresholds
- backoff timing
- bypass timing
- side-follow PD tuning
- reacquire gating timing
- UART debug print divider

## Debug Mode

When debug UART output is enabled, `main.c` prints a startup banner and can print:

- state transitions
- avoid-state transitions
- throttled sensor/status lines

Typical fields include:

- `lineCount`
- `lineErr`
- `usL_cm`
- `usR_cm`
- `irC_mm`
- `bumpMask6`
- obstacle confidence
- ramp confidence
- selected bypass side
- line-follow mode
- current avoid step

### Example Serial Output

```text
MODE DEBUG
STATE FOLLOW -> OBS_CONFIRM
AVOID IDLE -> OBS_CONFIRM
SENS st=BYPASS av=BYPASS lc=2 le=-120 usL=18 usR=24 ir=131 bump=0x00 obs=5 ramp=0 by=2 lf=0 step=6
```

### Serial Terminal Settings

Use the LaunchPad backchannel serial port with:

- **Baud:** `115200`
- **Data bits:** `8`
- **Parity:** `None`
- **Stop bits:** `1`

## Calibration and Verification

The project already includes dedicated support for bring-up and sensor validation.

### Calibration

`calibration.c` is used to gather:

- white/black reflectance counts
- line thresholds
- ultrasonic baselines
- latest sensor snapshots for CCS watch

### Verification

`verify.c` supports focused front-sensor checking for:

- bump mask decoding
- bump index identification
- multiple bump detection
- front IR raw verification

### Demo Mode

`demo.c` provides a staged system walkthrough with phases such as:

- startup
- sensor check
- line follow
- avoid check
- collision check
- emergency check

## Reacquire Gating

The obstacle path includes a dedicated `LINE_REACQUIRE_GATED` state intended to avoid snapping back onto an incorrect line segment immediately after bypass.

The current codebase already exposes debug/watch variables for:

- entry heading
- current heading
- yaw delta
- line accept/reject reason
- reacquire accept count
- sweep stage
- step timer
- front minimum range
- gating pass/fail status

This keeps tuning practical inside CCS without rewriting the line follower.

## CCS Watch Variables

The project exposes many globals specifically to make tuning easier in CCS.

Useful examples:

### Line Follow

- `gLineFollowMode`
- `gLineFollowMask`
- `gLineFollowCount`
- `gLineFollowErr`
- `gLineFollowCorr`
- `gLineFollowRampConf`

### Avoidance

- `gAvoidState`
- `gAvoidObsConf`
- `gAvoidObsConfirmCount`
- `gAvoidBumpMask6`
- `gAvoidBumpZone`
- `gAvoidChosenBypassSide`
- `gAvoidBoundaryModeActive`
- `gAvoidFrontBlocked`
- `gAvoidYawDelta`
- `gAvoidReacquireGatePassed`

### Demo / Verify / Calibration

- `gDemoStage`
- `gVerifyBumpMask`
- `gCalibrationData.*`

## Development Notes

- The project is written for **NoRTOS** operation.
- Timing is based on **SysTick**.
- The fast path is kept in plain C with compile-time feature switches.
- Low-level driver modules are intentionally separated from behavior modules.
- The motor driver remains isolated in `motor.c/.h`.

## Recommended Workflow

1. Start in calibration mode and verify thresholds.
2. Test line following on a clean track.
3. Enable debug mode when tuning obstacle behavior.
4. Watch CCS globals while testing bypass and reacquire logic.
5. Switch back to real-system mode after tuning.

## Safety

Before running on the floor:

- verify motor direction on a stand
- confirm bump switches are active and mapped correctly
- confirm ultrasonic readings are sane at short and medium range
- confirm line thresholds after calibration
- keep first obstacle tests at low speed

## Repository Status

This project has evolved beyond the default CCS placeholder and is now a structured robotics control codebase for TI-RSLK MAX.

The current README is intended to serve as the GitHub landing page for:

- hardware overview
- software architecture
- configuration entry points
- debug workflow
- bring-up and tuning

## License

No license file is currently included in this repository. Add one before public distribution if you plan to publish the project openly.
