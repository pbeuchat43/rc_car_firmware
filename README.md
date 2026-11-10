# RC car firmware

Pico firmware for an RC-style drive ESC, steering servo and research camera pan servo. It owns the actuator source selection, receiver handover and command deadlines. It speaks [RC Car Protocol v1 (RCPv1)](docs/rcpv1.md) over USB. See the [design](docs/design.md).

## Build

Use Pico SDK and picotool 2.3.1 with the TinyUSB submodule initialized and the ARM cross compiler installed. Both Waveshare RP2040-Zero and RP2350-Zero are build targets. Provide a reviewed profile header. The example has GPIO12/13 for RC drive/steering, GPIO14/15 for the output pair and GPIO7 for pan. Its timings and pulse limits are starting values for bench qualification.

~~~sh
cmake -S . -B build/rp2040 -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -Dpicotool_DIR=/path/to/picotool/install/lib/cmake/picotool \
  -DPICO_BOARD=waveshare_rp2040_zero \
  -DRC_CAR_PROFILE="$PWD/profiles/research_car.example.hpp"
cmake --build build/rp2040 --target rc_car_firmware_bundle
~~~

For RP2350, use a separate build directory and set PICO_BOARD to waveshare_rp2350_zero. The bundle target writes the UF2 and a manifest containing board, protocol, profile ID, build ID and artifact SHA-256. Save the selected profile and manifest with deployment records.

Verify the actual harness, signal levels, common ground, neutral behavior and output limits. Flash through physical BOOTSEL with actuator power isolated. Qualify receiver behavior with the transmitter off and back on, verify timing on the PWM pins and verify the ESC's operational neutral before connecting propulsion.

## Tests

~~~sh
cmake -S . -B build/native -DRC_CAR_NATIVE_TESTS=ON
cmake --build build/native
ctest --test-dir build/native --output-on-failure
~~~

Native tests simulate RC input and USB records. Physical acceptance includes the checks in the design.
