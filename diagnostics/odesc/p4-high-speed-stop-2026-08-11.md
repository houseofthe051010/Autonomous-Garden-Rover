# P4 high-speed stop diagnosis (2026-08-11)

Source log: `p4-blackbox-before-7000rpm-2026-08-11.log`

The retained P4 log shows successful starts at 72.99, 79.99, and 81.99
turns/s followed by explicit `STOP M0 complete` actions. It does not record an
axis, motor, or sensorless-estimator fault during those runs. The live status
afterward also reported zero UART hardware errors and zero query failures.

The previous P4 firmware limited commands to the lower of the saved ODESC speed
limit and a continuously recalculated `0.8 * VBUS * 170 KV` estimate. Battery
sag could therefore make an already-held command invalid. The browser treated
that rejected keepalive like a transport failure and issued STOP. A mobile
long-press could also generate `pointercancel`, producing the same explicit
stop sequence.

The prepared replacement uses a fixed 7,000 RPM (116.67 turns/s) P4 command
guard. The voltage-based 80-percent estimate remains informational. ODESC's
saved limits and all electrical, thermal, watchdog, and estimator protections
remain active. The replacement also logs command-guard rejections, suppresses
mobile selection/callout behavior on hold controls, and adds 60 seconds of
in-memory speed/current/temperature/power/voltage history.

This diagnosis does not prove that every historical STOP had the same cause;
normal button release also intentionally logs `STOP M0 complete`. A future
unexpected stop should be correlated with the new `GUARD`, `FAULT`, `DEADMAN`,
and UART events.
