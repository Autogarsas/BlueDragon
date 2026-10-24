# BlueDragon credits

BlueDragon combines existing projects. Its name identifies this integration; it
does not replace the authorship, licenses, or attribution of the upstream work.

| Project | Contribution |
| --- | --- |
| [dragonpilot](https://github.com/dragonpilot/dragonpilot/tree/0.11.1) | Base system, user interface, settings, ALKA, longitudinal features, and device support. Thanks to Rick Lan and the dragonpilot community. |
| [BluePilot](https://github.com/BluePilotDev/bluepilot/tree/bp-5.0) | Ford curvature steering, predicted-curvature blending, lane-change tuning, optional lane positioning, and Edge vehicle definitions adapted for this port. Thanks to BluePilotDev and contributors. |
| [sunnypilot](https://github.com/sunnypilot/sunnypilot) | Upstream work carried through BluePilot. Thanks to Haibin Wen, SUNNYPILOT LLC, and contributors. |
| [openpilot](https://github.com/commaai/openpilot), [opendbc](https://github.com/commaai/opendbc), and [panda](https://github.com/commaai/panda) | Underlying driving, vehicle-protocol, and hardware-interface software. Thanks to comma.ai and contributors. |

BlueDragon's integration includes the dragonpilot adapter, bounded Ford safety
mode, settings registration repair, matching H7 application packaging, and
diagnostic tools. It is an independent fork, not an official release or an
endorsement by any upstream project.

## Source versions

- dragonpilot 0.11.1 base: `3e8edb7e3d6d8fb0393f0604ee3954ee328857f5`.
- Original prebuilt source: `c0ab3550eca2e9daf197c46b7e4b24aa9637cf2e`.
- BluePilot bp-5.0: `c6f8350f17c0622d7a075fe03ae760425e68fd2c`.
- Preserved integration and H7 repair: `37666613d40505461f457c002a4fdd40686b9d89`.

## License notices

Original copyright notices and license files remain in place. See [LICENSE](LICENSE),
[LICENSE.md](LICENSE.md), [LICENSE_BLUEPILOT.md](LICENSE_BLUEPILOT.md), and licenses
within the included dependencies. These components do not all share one license.

This software is licensed under a custom license requiring permission for use.

This project uses software from Haibin Wen and SUNNYPILOT LLC and is licensed
under a custom license requiring permission for use.
