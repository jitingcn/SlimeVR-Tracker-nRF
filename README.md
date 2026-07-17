# SlimeNRF tracker firmware

Zephyr/NCS firmware for SlimeNRF motion trackers on Nordic nRF52 and nRF54L SoCs.

Official usage docs for the broader SlimeNRF ecosystem live at [docs.slimevr.dev/smol-slimes](https://docs.slimevr.dev/smol-slimes).

## Project history

This firmware is originally based on [SlimeVR/SlimeVR-Tracker-nRF](https://github.com/SlimeVR/SlimeVR-Tracker-nRF). Thanks for their work and for maintaining the project.

This repo was forked from [LyallUlric/Stacked-SmolSlime](https://github.com/LyallUlric/Stacked-SmolSlime) (stacked Promicro builds on top of the official tracker firmware), not from upstream `main`. When Stacked-SmolSlime stopped being maintained, We continued development here.

A hard fork followed because upstream `main` history was hard-reset, development direction was unclear and slowly, and upstream discouraged AI agent coding. This fork keeps history from before that reset and continues development. It started from my personal gaming needs. It now also covers many requests from the community and from Chinese DIY SlimeVR-compatible tracker builders.

Changes are driven by real playtesting, user needs, and normal software engineering practice. Untested patches are not accepted on that basis alone.

## Major differences from upstream

- Remote command from receiver side
- Tweaked VQF fusion parameters and backend
- Add mag support properly for ICM45686/LSM6DSV and QMC6309 (other mags are not well tested)
- More sensor calibration options and features
- Custom sdk-nrf fork with more features and fixes
- Different TDMA radio scheduling, clock sync methods
- OTA support via BLE/ESB
- Configurable IMU/mag sensor driver whitelist
- Resting state policy refinements, Different WoM policy
- Raw sensor data collection and analysis
- Experimental EqF fusion backend
- More community board vendors and Promicro variants

## License

Unless otherwise specified, all code in this repository is dual-licensed under either:

- MIT License ([LICENSE-MIT](LICENSE-MIT) or https://opensource.org/license/mit/)
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or https://opensource.org/license/apache-2-0/)

at your option. This means you can select the license you prefer!

Unless you explicitly state otherwise, any contribution intentionally submitted for
inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual
licensed as above, without any additional terms or conditions.
