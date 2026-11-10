# RC car firmware and vehicle interface

Approved design · 6 November 2026

The research platform needs a predictable way to command a PWM-controlled RC car and its camera pan servo. The Pico selects the drive and steering source, checks command freshness, and generates PWM. The ROS 2 interface accepts experiment commands and reports what the Pico selected. A change in qualified RC input cancels autonomy locally, even if the host is stalled or disconnected from ROS.

## Repository names and ownership

| Pair | Fit |
| --- | --- |
| **rc_car_firmware / rc_car_interface** | Recommended. Names the vehicle class and leaves the MCU and ROS details to the documentation. |
| rc_car_pico_firmware / rc_car_ros_interface | Makes both implementation platforms visible in the repository names. |
| rc_vehicle_firmware / rc_vehicle_interface | Leaves room for other vehicles with the same drive and steering contract, at the cost of a less specific name. |

- **Firmware** owns board support, PWM capture and output, authority, the control deadline, the wire specification, and a generic maintenance pulse mechanism.
- **Interface** owns exclusive USB connection, the host protocol client, ROS messages and launch, upstream command freshness, and publishing observed status. It runs independently on the collection's ROS 2 Jazzy baseline.
- **robot_deployment** selects firmware artifacts, vehicle profiles, connected devices, namespaced launch, and supported experiment settings. It owns ESC model recipes, the calibration wizard, physical procedures, and integration evidence.
- **robot_runtime** supervises the launched node and may expose declared status and authorized mode requests. Its process lifecycle and operation results stay distinct from Pico authority. The camera and detector components remain independent ROS components. A deployment can connect its own experiment controller to the vehicle and pan endpoints.

~~~mermaid
flowchart LR
    E[Experiment controller] -->|Drive and steering targets| H[ROS vehicle interface]
    H <-->|Session commands and status| P[Pico authority and PWM]
    R[RC receiver] -->|Two input pulses| P
    P -->|Two output pulses| V[ESC and steering servo]
    P -->|Pan pulse| C[Camera pan servo]
    D[Deployment calibration tool] -. exclusive maintenance session .-> P
~~~

## Vehicle and board profile

Support cars whose ESC and steering servo accept independent RC-style pulse widths. Start with one qualified Pico board and harness. Add board variants through pin and timing configuration plus bench evidence. The board definition validates GPIO assignments, electrical compatibility and level conversion, PWM slices, and timing. Hardware PWM emits continuous frames, initially 50 Hz. Stage drive and steering together for the same frame. Keep pan on a separate output timer or slice. A neutral drive pulse requests the ESC's neutral behavior, which the deployment must verify for the selected ESC.

A deployment-owned vehicle profile supplies each RC input's calibrated min, center, max, polarity and neutral deadband, and each actuator's output min, neutral, max and polarity. The drive channel also has a separate board/profile-approved maintenance envelope. Operational limits can be narrower than ESC calibration endpoints. For v1, compile validated board and vehicle settings into the deployment-selected firmware artifact. This makes neutral and limits available before USB and when running manually. The artifact and the firmware identity report the effective profile ID. Changing these limits requires a new artifact. Experiment-specific shaping within the limits belongs in the experiment controller.

Commands represent normalized actuator demand. Use −1 to +1 for each axis, with zero mapped to configured neutral. Positive drive means forward demand, positive steering means left, and positive pan means left when viewed in the vehicle's forward direction. Profile polarity handles the wiring and actuator direction. Reverse is an ordinary signed demand. The ESC determines whether that demand brakes or reverses. Use one piecewise-linear mapping around center, with a single configured RC neutral deadband and integer PWM rounding. Reject malformed or out-of-range host inputs. Clamp valid RC input at calibrated travel and enforce the physical output envelope in the final PWM path for every source.

## RC quality and authority

The Pico qualifies both RC input channels independently. A channel is valid when it supplies complete pulses inside its electrical width and period windows. Use a hardware-timed capture path and qualify recovery on three consecutive valid periods per channel. A completed invalid sample, capture fault, or age of at least 100 ms marks that channel bad. Initial bench settings are 750–2250 µs for width and 10–35 ms for period, subject to measured receiver output. Validate that calibrated travel lies within the electrical window. An overflow or capture service gap invalidates queued history. Only newly observed complete pulses restore freshness.

Combined RC is **GOOD** when both channels are qualified and fresh and **BAD** otherwise. After reset the Pico reports **UNKNOWN** while establishing its baseline. The first qualified pair establishes GOOD immediately. If there is no qualified pair by 150 ms, establish BAD. The initial baseline is not an edge and autonomy cannot start while UNKNOWN. Afterwards, every qualified GOOD ↔ BAD transition increments the authority generation and cancels any active autonomy. RC stick values and direction do not cancel autonomy. The first vehicle is expected to stop emitting PWM on both channels when its transmitter turns off. Verify this at the Pico inputs. A receiver that emits valid-looking neutral or held pulses requires a qualified receiver configuration or another link indication for this behavior. A PWM validity check alone cannot distinguish those pulses from live operator commands. [Example manufacturer failsafe behavior](https://traxxas.com/sites/default/files/58076OM-EN-R00.pdf#page=14)

| Effective state | Drive and steering | Entry and exit |
| --- | --- | --- |
| **Neutral** | Configured neutral pair | Start here. Also use this state when manual is selected and RC is UNKNOWN or BAD, or while waiting for the throttle to enter the neutral band. |
| **Manual** | Both qualified RC inputs | The normal fallback, with or without USB. If RC turns BAD, enter Neutral. |
| **Autonomous** | Fresh host demand and the configured RC contribution on each axis | Enter only on an explicit, current-generation request with a current session. Stable GOOD or BAD can start autonomy. Output neutral while awaiting the first new drive target. A qualified RC edge, command expiry, session loss, or explicit mode change ends this run. |
| **Hold** | Configured neutral pair, independent of RC | Explicit host inhibit or a firmware fault. Survives USB loss. An explicit Manual request releases an operator hold. Fault holds require fault clearance and the appropriate recovery or reset. |
| **Maintenance** | Steering neutral, drive neutral or the permitted maintenance pulse | Exclusive maintenance while the operating node is stopped. On exit return to Hold. |

Normal autonomy cancellation returns to the manual selection. **On BAD → GOOD, both outputs stay neutral until a fresh valid throttle sample is in the calibrated neutral band while combined RC is GOOD. One sample is enough. There is no dwell.** The sample that establishes GOOD may count. Once Manual is active the throttle can leave the band and drive normally. On GOOD → BAD both outputs go neutral and the manual entry condition resets. Apply the same manual entry condition at boot and when switching from Autonomous or Hold to manual, including after USB failure or drive-command expiry. A previous neutral sample from before the handover cannot satisfy this condition. Hold is the explicit way to keep both outputs neutral while the receiver is good. It is a firmware inhibit for this boot. A reset starts the boot sequence again, so power isolation is required for bench work.

The Pico is the sole writer of mode and output. Process an RC edge and check deadlines before admitting any simultaneous enable or drive command. Advance the authority generation on each authority change, including Autonomy entry or exit and Hold or Maintenance entry or exit. An RC edge always advances the generation, even when the effective mode stays the same. One transition can satisfy both conditions. Clear the autonomous target on exit. Every Autonomy request compares the generation from a recent status. A subsequent drive command uses the new generation reported after entry. A queued request from before an RC edge or other authority change cannot restore autonomy. A double edge between status reports still changes the generation and leaves the last cancellation reason visible. Reconnect, fresh drive packets, or a resumed ROS publisher do not re-enable autonomy. Use a fresh explicit request.

Two alternatives were considered for the RC interlock. Putting it in the interface allows quicker policy changes but makes cutoff depend on telemetry, USB delivery and host scheduling. Duplicating the rule in both layers creates two decisions that can disagree. Put the interlock on the Pico and let the interface surface the observed result and the requested mode.

### Independent experimental control

Drive and steering each have a three-way selector. Both default to Autonomy. Compute centered, deadbanded RC demand and validated host demand in the same normalized units. Combine each axis once before converting it to PWM.

| Axis choice | Stable GOOD | Stable BAD |
| --- | --- | --- |
| **RC** | RC demand | Neutral |
| **Autonomy** | Host demand | Host demand |
| **Add RC to autonomy** | clamp(host + RC, −1, +1) | Host demand, with zero RC contribution |

For example, RC drive with autonomous steering lets a person control throttle during a steering experiment. Additive steering lets a person correct the controller's steering demand on the same axis. Either qualified RC edge cancels the entire run, regardless of the selections. The BAD column applies when a run was explicitly started with RC already BAD. Selecting RC on both axes maps to a Manual request, including the manual throttle entry check.

Selections are part of the explicit Autonomy request. Applying different selections mid-run starts a new generation and clears the current target. The Pico stages neutral while waiting for the new target. Status reports the configured choices, actual selected sources, and which axes saturated before PWM mapping. To hold just one axis at neutral during an experiment, select Autonomy on that axis and send zero. This needs no fourth selector choice.

### Camera pan

Pan commands are independent of ordinary vehicle mode, RC quality, and the drive lease. At boot leave the output low until the configured servo startup delay, then emit center for at least one PWM frame. Hold the most recently programmed pan target through ordinary vehicle mode and USB changes. Reboot clears the target and returns to the startup behavior. Clear unsent targets on a session change. During Maintenance, freeze the output or keep it disabled if startup has not completed, and reject new pan commands. Report PWM output and readiness as commands to the servo. Pan traffic, loss, or rejection does not refresh or cancel drive authority.

## USB contract

Use one USB CDC data stream and an exclusive host client. The firmware repository owns the versioned wire spec and literal accepted/rejected fixtures. Both implementations check the same fixtures. Pick a configured stable USB identity when multiple compatible devices are connected, and verify the identity over the open port before sending commands that can change state. If no selector is configured, discovery succeeds only when exactly one compatible device is present. Treat an unexpected role, version, device or required profile as unavailable.

RC Car Protocol v1 (RCPv1) uses bounded fixed-order ASCII. It can be inspected on a bench without tooling, and a single status replaces the need to correlate several records. Reconsider binary framing only if measurement shows a constraint the compact ASCII format cannot meet. Each record has the form @RCPv1,TYPE,fields...,CRC followed by LF. Maximum 256 bytes including LF. CRC-32/ISO-HDLC is eight uppercase hex characters over the bytes from @ through the comma before CRC. IDs and challenges are 16 uppercase hex characters. Integers use shortest decimal form with no leading zeros, plus sign, or negative zero. Use a positive uint32 sequence for each mutating host record and nonnegative uint64 for MCU time in milliseconds and authority generation. Normalized wire targets are integers from −10000 to +10000, rounded once by the host to nearest with ties away from zero. Other numeric values are nonnegative unless specified. Use only the defined ASCII tokens and exact field counts.

| Type | Direction | Fields after type, in order |
| --- | --- | --- |
| HELLO | Host → Pico | challenge |
| ID | Pico → host | echoed challenge, session, device ID, boot ID, role, board, build ID, profile ID |
| MODE | Host → Pico | session, seq, expected generation, status tick, requested mode, drive choice, steer choice |
| DRIVE | Host → Pico | session, seq, generation, status tick, drive target, steer target |
| PAN | Host → Pico | session, seq, status tick, pan target |
| RESULT | Pico → host | session, request seq, result code, resulting generation |
| STATUS | Pico → host | session, tick, generation, effective mode, state reason, last cancellation generation, last cancellation reason, RC state, valid-axis mask, RC drive µs, RC steer µs, requested drive choice, requested steer choice, selected drive source, selected steer source, saturated-axis mask, latest accepted drive seq, selected drive seq, drive PWM µs, steer PWM µs, programmed pan seq in this session, pan PWM µs |
| MAINTENANCE | Host → Pico | session, seq, expected generation, status tick, operation, drive pulse µs |

The role token is RCCAR. Board, build and profile tokens use 1–24 characters drawn from letters, digits, hyphen and underscore. Modes on the wire are NEUTRAL, MANUAL, AUTO, HOLD and MAINTENANCE. MODE requests accept MANUAL, AUTO and HOLD. Axis choices and selected sources are HOST, RC, SUM and NONE. NONE is reserved for neutral output or a choice outside AUTO. For MANUAL or HOLD requests both choice fields are NONE. The host sends MANUAL for an RC/RC selection. The Pico rejects AUTO with RC/RC. RC state is UNKNOWN, GOOD or BAD. Both axis masks use bit 0 for drive and bit 1 for steering. Use zero for unobserved RC widths, unavailable pan PWM or sequences with no corresponding accepted/programmed value. A held pan PWM can be nonzero with pan sequence zero after a session change. The selected drive sequence is zero whenever no host drive target supplies an axis. STATUS reports the PWM values staged by the output driver in the same decision snapshot as authority and selected sequences. A result or PWM register value provides no measurement of the signal or movement. RESULT applies to MODE and MAINTENANCE. Success means the Pico accepted the transition or operation. A subsequent STATUS confirms the observed state and staged outputs. Enumerate result and reason codes in the wire contract before implementation. Use tokens of at most 20 ASCII characters, including reasons for waiting for neutral throttle, RC edges, command expiry and link loss.

~~~text
@RCPv1,HELLO,0123456789ABCDEF,17379CD6
@RCPv1,DRIVE,0123456789ABCDEF,2,7,1234,-2500,500,8B7EBE4C
~~~

- Issue a fresh challenge on open. An ID echoes it and binds a fresh session token. Repeating the same challenge in the live session is idempotent. A new challenge, USB loss, or a transport service gap of at least 100 ms invalidates the old session and autonomous authority. A new session clears sequence history and pending transport data. Output selection does not wait for USB.
- Strictly increasing accepted sequence numbers are scoped to the session. Retransmitted records cannot reapply a target or renew its age. Establish a new session before exhaustion. A MODE or MAINTENANCE mutation checks the generation in the same authority step that applies it. A valid current-session HOLD may ignore a stale generation to let the operator inhibit promptly. While Maintenance is active, HOLD aborts the maintenance output and latches neutral.
- Mutations echo a recent STATUS tick from that session. Reject future ticks or ticks aged 150 ms or more on the MCU clock. This bounds stale serial input without synchronizing host and MCU clocks. Drop an incomplete frame after 100 ms and discard malformed or oversize frames through LF. Such frames do not update sequences, targets or leases. Send no per-target ACK. A missing transition RESULT is resolved against fresh status. An unresolved enable must not be replayed automatically.
- Publish STATUS at 25 Hz while connected. The interface sends the latest fresh drive target at up to 50 Hz, never a queued series of historical targets. When the Pico accepts AUTO, allow up to 500 ms to receive the first new-generation drive target while holding neutral. Once the stream starts, its lease expires at 100 ms from the last accepted DRIVE. Status, HELLO, pan and maintenance traffic cannot renew it.
- Use bounded work and buffers. The control path services capture, deadlines and paired PWM independently of USB. USB backpressure may drop an unsent status snapshot but cannot delay actuator decisions. Never splice partial frames. A monotonic actuator loop and hardware watchdog detect missed service. Start with one nonblocking loop plus hardware capture and PWM. Isolate USB on another core only if the single-core implementation cannot meet the timing checks. [Raspberry Pi PWM and watchdog APIs](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html)

## ROS 2 contract

Use one namespaced rclcpp node with locally defined actuator messages. The interface never reinterprets RC health or decides the physical source. It mirrors the current session and generation, clears pending commands on generation change or reconnect, and exposes the Pico's state. Keep the serial loop bounded and independent of ROS publication. Use one latest-value mailbox for each target and a separate mode operation with bounded time to complete. The process can remain running with no device and report unavailable.

| Private name | Contract |
| --- | --- |
| ~/command | Stamped drive and steering pair and current session/generation. Only a producer with current status can send a target for the current run. One command producer per run. |
| ~/pan_command | Stamped normalized pan target and current session. One latest value, independent of the drive stream. |
| ~/set_mode | Request Manual, Autonomy or Hold. Supply a request timestamp and the session and generation seen by the caller. Autonomy includes an independent RC, Autonomy or Sum choice for each axis. Reply with the correlated MCU result or an explicit unavailable, busy or uncertain outcome. |
| ~/status | One timestamped snapshot of connection/freshness and MCU status. Report session, generation, mode and reason, last cancellation, RC validity, requested and selected sources by axis, saturation, accepted and selected sequence, and programmed PWM. Distinguish unobserved or stale values from neutral. |
| ~/diagnostics | Low-rate connection, compatibility, frame errors, and timeouts. |

Use volatile depth-one command and status topics. Use reliable delivery for mode service requests and responses, with volatile durability and application-level freshness checks. For drive and pan, use best effort to favor the latest target. Reject stale or implausibly future stamped commands and mode requests, with a proposed 200 ms source age and 20 ms future tolerance on a shared system clock. Physical actuation uses live time. Convert remaining age at receipt to a monotonic deadline and stop forwarding at expiry, even if the ROS clock subsequently stalls. A clock discontinuity clears the input cache. Report link and status freshness independently, and stop sending drive if status has been unavailable for 150 ms or a local transport scheduling gap makes its queue age unknown. The Pico's clock still determines drive expiry and RC transitions. [ROS 2 Jazzy QoS policy definitions](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)

## ESC calibration and flashing

Place the Traxxas wizard in **robot_deployment**. It reserves the maintenance window, verifies the vehicle interface is stopped and cannot be restarted during the operation, opens the exact device exclusively, checks the profile, and guides the operator through the selected ESC's prompts and endpoint sequence. Use the host protocol client for the direct USB exchange. The ESC recipe owns LED/button instructions and timing. Firmware implements a generic pulse mechanism. The ROS node exposes ordinary control and status.

1. Mechanically prevent drive before opening the maintenance session, using the selected ESC model's procedure. Enter Hold and confirm neutral for two output periods. The tool explicitly enters Maintenance on the live session. Firmware requires RC drive to be absent/bad or in its neutral band at entry and while operating.
2. MAINTENANCE operations are ENTER, PULSE, ABORT and EXIT. PULSE requests one raw drive width inside the approved maintenance envelope. Other operations send zero for the unused pulse field. The tool begins at profile neutral and refreshes the selected width at least every 100 ms while guiding the sequence. Use a 500 ms MCU pulse lease, a 15 s continuous nonneutral limit, and a 120 s maximum maintenance session as initial bounds. Repeated nonneutral writes do not restart the continuous dwell limit. Require at least two full neutral frames before a new nonneutral dwell can begin after a neutral interval. The programmed drive neutral must be the same value used for ordinary boot, manual and autonomous operation.
3. Abort, expiry, RC leaving its permitted condition, or disconnect returns drive to neutral and latches Hold. Old maintenance packets cannot restart output. EXIT is explicit and returns to Hold with steering still neutral. The wizard verifies the staged outputs and asks the operator to confirm ESC acceptance before returning to normal use. Record the device, profile, endpoints, and the physical observation. Firmware cannot read the ESC's stored settings. [Example Traxxas XL-5 programming instructions](https://traxxas.com/sites/default/files/KC1761-R01_3018R-XL5_INST_140430-EN.pdf)
4. Flash with actuator power isolated using the selected board's physical BOOTSEL procedure. Deployment verifies board and artifact identity, and verifies the application after reboot. Keep flashing and reset commands outside the operational wire protocol.

## Acceptance

- Run host and firmware state tests for startup with GOOD, BAD and UNKNOWN, both qualified edges during autonomy, one failed axis, re-enable while stable BAD, unchanged reverse input, two edges between status samples, and races between the edge, a delayed enable and a buffered drive command. Cover all axis combinations with stable GOOD and BAD, including RC/RC mapping to Manual, positive and negative saturation, zero RC contribution under BAD, and replacement of selections during a run.
- Test shared golden frames, maximum sizes, fragmentation, bad CRC, duplicate/out-of-order sequences, old sessions and generations, stale status ticks, dropped RESULT, MCU reboot, and session recovery. Use a fake MCU or a pseudo-terminal to exercise the ROS launch and serial state transitions without a vehicle.
- Scope physical PWM during reset, first command, RC failover, the drive deadline, USB stalls, and host death. With the initial 50 Hz profile, target at most 22 ms from a qualified edge or lease expiry to the first updated PWM frame (2 ms decision budget plus 20 ms frame period). For PWM loss, target at most 125 ms from the last valid pulse to neutral. Measure the separate BAD → GOOD qualification delay. Confirm reverse never changes authority, off-center throttle delays Manual after recovery, and the first eligible in-band throttle sample enables Manual without a dwell.
- Verify pan startup, motion limits, hold across USB/authority changes and neutral after reset. Exercise powered ESC programming only on the prepared bench, confirm the ESC's completion indication and operational neutral, and separately verify pulse widths. Qualify each new receiver/harness/profile, including transmitter loss and recovery, before enabling that vehicle in a deployment.

Both Waveshare RP2040-Zero and RP2350-Zero are build targets. The example settings and timing targets require physical qualification before selecting a production profile.
