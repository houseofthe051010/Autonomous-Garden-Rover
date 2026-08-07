# Contributing

## Scope

Changes should keep the ESP32-P4 firmware as the primary rover host. Raspberry
Pi and commissioning projects under `legacy/` are maintained as reference
material and should not become dependencies of the active firmware.

## Development workflow

1. Open an issue for changes that alter wiring, protocols, safety behavior, or
   persistent configuration.
2. Make focused changes in one subsystem where possible.
3. Build the affected target using the instructions in its README.
4. Test hardware at low speed with the mechanism unloaded or raised.
5. Document the board revision, wiring, firmware version, and observed result.

Do not commit generated build directories, local `sdkconfig` files, Python
caches, Wi-Fi credentials, private controller keys, full-flash backups, or
serial captures containing secrets.

## Compatibility

Protocol changes must document framing, units, ranges, asynchronous messages,
timeouts, and watchdog behavior. Prefer backward-compatible additions. When a
breaking change is unavoidable, update both endpoints and the protocol document
in the same change.

## Safety

Motor commands require bounded tests, a working stop path, and timeout behavior.
Do not weaken watchdogs or startup-safe output states without documenting the
reason and adding equivalent protection.

## Licensing

Contributions to original project code are accepted under the MIT License.
Derived Marlin files and other third-party material retain their upstream
licenses and notices. Do not copy code into this repository unless its license
permits redistribution.
