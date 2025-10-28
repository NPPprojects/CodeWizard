# LED Gem Strip + Joystick Integration Plan

## Goals
- Combine the LED Gem Strip lighting demo with the joystick input demo
- Use joystick state to select lighting modes and control parameters
- Preserve standalone functionality to aid testing and debugging

---

## 1. Recon the Existing Samples
- Review `led_gem_strip` source: identify init sequence, update loop, lighting mode abstractions, hardware dependencies.
- Review joystick sample: note input reading cadence, scale/axis mappings, button handling, debounce and calibration logic.
- Document shared dependencies (board setup, SDK libs, hardware pins) and any conflicting configuration.

## 2. Establish Combined Project Skeleton
- Create new project directory (or branch) with shared build configuration.
- Import both sample sources; normalize include paths and CMake/Makefile targets.
- Define config header for pin mappings so hardware settings live in one place.

## 3. Harmonize Initialization
- Merge board/peripheral init routines ensuring GPIO, I2C/SPI, timers, and interrupt config do not conflict.
- Decide on unified main loop structure (event-driven vs. polling) and document timing assumptions (e.g., joystick poll rate vs. LED refresh).

## 4. Design Mode Mapping
- List candidate lighting modes from LED sample.
- Choose joystick controls:
  - Axes: map to brightness, color hue, speed, etc.
  - Buttons: cycle through modes, toggle power, trigger special effects.
- Define state machine (enum + struct) describing current mode and parameters.

## 5. Implement Input Handling Layer
- Wrap joystick reading in reusable module: calibration, dead zones, filtering.
- Expose clean API (e.g., `JoystickState getJoystickState();`) for main loop consumption.
- Add diagnostic logging or LED blink codes for troubleshooting input anomalies.

## 6. Integrate Lighting Control
- Adapt LED update function to accept dynamic parameters from joystick state.
- Ensure timing-safe updates: double-buffer if needed, and respect existing refresh limits.
- Add guardrails for invalid inputs (clamping, fallback modes).

## 7. Testing Strategy
- Unit test joystick module if framework available; otherwise create lightweight sanity checks.
- Build firmware variants: (a) joystick only, (b) LEDs only, (c) combined, to isolate regressions.
- Bench test on hardware: verify each joystick action maps to intended lighting behavior.

## 8. Documentation & Next Steps
- Update README with build instructions, dependencies, hardware wiring diagram.
- Add troubleshooting tips (e.g., joystick drift, LED power issues).
- Plan future enhancements: smoothing filters, additional modes, configurable profiles.
