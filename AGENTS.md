# Repository guidance

Firmware owns actuator authority, board and PWM behavior, and the RCPv1 wire contract. Read README.md, docs/design.md and docs/rcpv1.md for the active contracts. Keep the codec and literal fixtures synchronized with rc_car_interface when making a coordinated protocol change.

Develop on a feature branch from dev. Keep main for agreed baselines. Run the native suite and build both Waveshare targets for changes to shared firmware, wire behavior or build settings. Distinguish build and simulation evidence from physical acceptance. Hardware tests require an identified vehicle, board, profile, harness and procedure.
