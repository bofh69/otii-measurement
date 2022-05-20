# otii_measurement

A tool to meassure average current with a Qoitech Otii instrument.

# Please Note

This is not approved or supported by Qoitech in any way.

Only tested with Otii FW 1.1.2 and with no access to the protocol
description.

# Functionality

The program is quite basic. It turns on the power, optionally waits
for one (or both) of the digital inputs to go high or low and then
measures the current over a specific time period.

The code is an almost direct port of an experimental python script so
not much though has been put into its design. The python code's serial
library was too slow running on a raspberry pi so this is the result.

Unexpected answers (like over/under voltage during calibration) is
silently ignored.

PRs are happily accepted.

## Example:

```sh
otii_measurement --debug output.log --calibrate --wait-for-GPI1 true \
                 --timeout 20 --measurement-time 60 \
                 --unit uA -v 3.3
```

This calibrates the Otii, waits for GPI to be true/high, then measures
the current for 60 seconds and reports it in micro ampere.

The communication log is written to output.log

## Cross compile for RPi
```sh
rustup target add armv7-unknown-linux-musleabihf

sudo apt-get install gcc-multilib-arm-linux-gnueabihf musl-tools
```

Add to `~/.cargo/config`:
```
[target.armv7-unknown-linux-musleabihf]
linker = "arm-linux-gnueabihf-gcc"
```

Build with:
```sh
CC=/usr/bin/musl-gcc cargo build --target armv7-unknown-linux-musleabihf --release
```

The file is then in ./target/armv7-unknown-linux-musleabihf/release/otii_measurement
