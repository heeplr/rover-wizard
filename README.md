# Repository Coverage

[Full report](https://htmlpreview.github.io/?https://github.com/heeplr/rover-wizard/blob/python-coverage-comment-action-data/htmlcov/index.html)

| Name                                             |    Stmts |     Miss |   Branch |   BrPart |   Cover |   Missing |
|------------------------------------------------- | -------: | -------: | -------: | -------: | ------: | --------: |
| rover\_wizard/\_\_init\_\_.py                    |       19 |        6 |        2 |        1 |     67% |     29-34 |
| rover\_wizard/\_\_main\_\_.py                    |       49 |        1 |       18 |        1 |     97% |        94 |
| rover\_wizard/boat\_defs.py                      |       33 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/cli\_connection.py                 |       15 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/context.py                         |      296 |        8 |      112 |        8 |     96% |108, 127-128, 134, 162, 206-\>208, 212-\>219, 293, 300, 304 |
| rover\_wizard/flow.py                            |       27 |        0 |        6 |        0 |    100% |           |
| rover\_wizard/log\_cli.py                        |      175 |       15 |       50 |        6 |     91% |74, 98-100, 196, 214-215, 218-219, 225-227, 229-230, 249-\>256, 258 |
| rover\_wizard/log\_storage.py                    |       58 |        1 |        6 |        1 |     97% |        90 |
| rover\_wizard/mavlink\_defs.py                   |      111 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/model.py                           |       85 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/motion/\_\_init\_\_.py             |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/motion/analysis.py                 |      146 |        3 |       34 |        4 |     96% |180, 233, 261-\>269, 313 |
| rover\_wizard/motion/driving.py                  |      178 |        5 |       48 |        2 |     97% |66, 238-239, 248-\>251, 339-340 |
| rover\_wizard/motion/ekf\_sensor\_errors.py      |      112 |        2 |       40 |        3 |     97% |116, 149, 165-\>163 |
| rover\_wizard/motion/gps\_heading.py             |       48 |        1 |       18 |        2 |     95% |50, 71-\>64 |
| rover\_wizard/motion/sail\_test.py               |       11 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/motion/servo\_test.py              |      291 |       10 |       82 |        7 |     95% |118-119, 123-124, 132-133, 272-\>274, 406, 445, 483, 496 |
| rover\_wizard/motion/step\_tuning.py             |      180 |        4 |       54 |        3 |     97% |169, 181, 356-357 |
| rover\_wizard/motion/vibration.py                |       23 |        0 |        6 |        0 |    100% |           |
| rover\_wizard/param\_cli.py                      |      120 |        5 |       40 |        2 |     96% |97, 122-124, 164 |
| rover\_wizard/rover\_defs.py                     |       74 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/session/\_\_init\_\_.py            |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/session/activity\_log.py           |       15 |        2 |        0 |        0 |     87% |     20-22 |
| rover\_wizard/session/changes.py                 |       24 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/session/store.py                   |       86 |        0 |       18 |        1 |     99% | 113-\>116 |
| rover\_wizard/steps/\_\_init\_\_.py              |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/accel.py                     |       72 |        7 |       18 |        5 |     87% |49, 65-66, 95, 104, 127, 146 |
| rover\_wizard/steps/barometer.py                 |       60 |        2 |       18 |        2 |     95% |   54, 101 |
| rover\_wizard/steps/battery.py                   |       52 |        0 |       14 |        1 |     98% |   52-\>57 |
| rover\_wizard/steps/beacon.py                    |       16 |        1 |        4 |        2 |     85% |19-\>exit, 22 |
| rover\_wizard/steps/boat\_loiter.py              |        6 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/compass.py                   |      293 |        9 |       88 |       12 |     94% |83, 96, 145, 184-\>195, 201, 248, 458, 466-\>468, 468-\>470, 473, 558, 579 |
| rover\_wizard/steps/compass\_drive.py            |       23 |        0 |        4 |        0 |    100% |           |
| rover\_wizard/steps/depth\_sounder.py            |       61 |        0 |       12 |        1 |     99% |111-\>exit |
| rover\_wizard/steps/direction\_check.py          |       66 |        3 |       26 |        6 |     90% |102, 108, 123, 130-\>134, 136-\>145, 141-\>145 |
| rover\_wizard/steps/distance\_sensors.py         |      173 |        9 |       52 |        8 |     92% |290, 297, 342-343, 360-\>350, 393, 401-402, 415-\>408, 421-422 |
| rover\_wizard/steps/ekf\_sources.py              |       61 |        2 |       10 |        2 |     94% |  101, 114 |
| rover\_wizard/steps/failsafe.py                  |       20 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/final.py                     |        4 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/gains.py                     |       69 |        4 |       20 |        6 |     89% |76, 83, 94-\>104, 101-\>97, 127, 133 |
| rover\_wizard/steps/generator.py                 |       15 |        1 |        4 |        2 |     84% |31-\>exit, 34 |
| rover\_wizard/steps/gimbal.py                    |       75 |        8 |       16 |        5 |     86% |89-\>92, 101, 117-118, 142, 152-153, 167-168 |
| rover\_wizard/steps/gps.py                       |      117 |        2 |       34 |        3 |     97% |140, 225, 227-\>exit |
| rover\_wizard/steps/gripper.py                   |       31 |        3 |        4 |        2 |     86% | 44-46, 58 |
| rover\_wizard/steps/log\_settings.py             |       12 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/motor\_output.py             |      112 |        5 |       34 |        6 |     92% |86, 127, 188-\>213, 191, 211, 222 |
| rover\_wizard/steps/navigation.py                |       17 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/optical\_flow.py             |        5 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/orientation.py               |       52 |        3 |       10 |        3 |     90% |25, 69, 85 |
| rover\_wizard/steps/osd.py                       |        7 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/predrive.py                  |       76 |        2 |       20 |        6 |     92% |43, 61-\>70, 82-\>89, 89-\>91, 98-\>115, 116-\>123, 183 |
| rover\_wizard/steps/rc.py                        |      101 |        4 |       28 |        3 |     95% |131-\>130, 159-160, 163-164 |
| rover\_wizard/steps/relays.py                    |       37 |        1 |        6 |        1 |     95% |        50 |
| rover\_wizard/steps/remote\_id.py                |       22 |        2 |        6 |        2 |     86% |    33, 37 |
| rover\_wizard/steps/rpm.py                       |       30 |        0 |        4 |        1 |     97% | 53-\>exit |
| rover\_wizard/steps/sails.py                     |       52 |        0 |       16 |        0 |    100% |           |
| rover\_wizard/steps/save.py                      |       12 |        0 |        4 |        0 |    100% |           |
| rover\_wizard/steps/scripting.py                 |       18 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/serial\_ports.py             |       20 |        0 |        4 |        0 |    100% |           |
| rover\_wizard/steps/servo\_outputs.py            |       43 |        2 |        8 |        3 |     90% |70-\>76, 74, 77 |
| rover\_wizard/steps/sprayer.py                   |       16 |        2 |        4 |        1 |     85% |     24-26 |
| rover\_wizard/steps/steering\_output.py          |       21 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/temperature.py               |       11 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/top\_speed.py                |       32 |        1 |        8 |        3 |     90% |55, 63-\>67, 67-\>73 |
| rover\_wizard/steps/torqeedo.py                  |       52 |        0 |       12 |        0 |    100% |           |
| rover\_wizard/steps/turning.py                   |       52 |        4 |       10 |        5 |     85% |65, 72, 96, 103, 111-\>116 |
| rover\_wizard/steps/util/\_\_init\_\_.py         |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/util/actions.py              |       19 |        2 |        8 |        3 |     81% |13, 18-\>exit, 25 |
| rover\_wizard/steps/util/boat\_hardware.py       |       12 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/util/choices.py              |        3 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/util/compass\_slots.py       |       27 |        2 |       10 |        2 |     89% |    40, 55 |
| rover\_wizard/steps/util/driving.py              |       15 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/util/ekf\_measurement.py     |       76 |        6 |       30 |        7 |     88% |44, 59, 74-\>79, 93-\>95, 95-\>97, 107-110, 126-127 |
| rover\_wizard/steps/util/heading\_calibration.py |       17 |        2 |        6 |        2 |     83% |    27, 33 |
| rover\_wizard/steps/util/outputs.py              |       89 |        1 |       40 |        1 |     98% |       243 |
| rover\_wizard/steps/util/ports.py                |       40 |        1 |        8 |        1 |     96% |        62 |
| rover\_wizard/steps/util/probing.py              |       36 |        0 |        6 |        1 |     98% |   65-\>67 |
| rover\_wizard/steps/util/rover\_hardware.py      |       78 |        0 |        8 |        0 |    100% |           |
| rover\_wizard/steps/util/straight\_runs.py       |       18 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/vehicle\_type.py             |       41 |        1 |       14 |        1 |     96% |        76 |
| rover\_wizard/steps/verify.py                    |      170 |        9 |       44 |        8 |     92% |101, 159-160, 229, 281, 296, 315-\>317, 324, 366-367 |
| rover\_wizard/steps/visual\_odometry.py          |        5 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/wheel\_encoders.py           |       57 |        5 |       18 |        5 |     87% |66, 75, 85, 95, 97 |
| rover\_wizard/steps/wind\_vane.py                |      108 |        0 |       28 |        1 |     99% | 209-\>211 |
| rover\_wizard/texts.py                           |       24 |        0 |        4 |        0 |    100% |           |
| rover\_wizard/ui/\_\_init\_\_.py                 |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/ui/connect\_screen.py              |       60 |        0 |        8 |        1 |     99% | 62-\>exit |
| rover\_wizard/ui/main\_screen.py                 |      482 |       18 |      162 |       17 |     95% |197-199, 243, 300-301, 395, 397-398, 492-\>483, 496-\>483, 504, 510, 533, 544-\>exit, 547-\>exit, 551, 555-\>exit, 578-\>exit, 591-\>exit, 644, 684, 698, 702-703, 717-\>exit |
| rover\_wizard/ui/markdown.py                     |        4 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/ui/modals.py                       |      125 |        8 |        8 |        2 |     92% |74, 196-202, 217, 220-\>exit |
| rover\_wizard/ui/monitor.py                      |       74 |        1 |       16 |        2 |     97% |72, 120-\>119 |
| rover\_wizard/ui/param\_docs.py                  |       68 |        0 |       20 |        0 |    100% |           |
| rover\_wizard/ui/param\_screen.py                |      247 |       16 |       56 |        8 |     92% |57-58, 141-\>exit, 146, 166, 204, 247, 368-\>exit, 375-376, 398-\>exit, 422-423, 431-433, 447, 455-456 |
| rover\_wizard/ui/step\_catalog.py                |       19 |        0 |        6 |        0 |    100% |           |
| rover\_wizard/ui/wizard\_app.py                  |       43 |        4 |        4 |        0 |     87% |     73-76 |
| rover\_wizard/util/\_\_init\_\_.py               |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/util/errors.py                     |        3 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/util/rotation.py                   |       59 |        3 |       18 |        3 |     92% |80, 106, 144 |
| rover\_wizard/util/timebase.py                   |       12 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/util/values.py                     |       22 |        1 |        8 |        1 |     93% |        32 |
| rover\_wizard/vehicle/\_\_init\_\_.py            |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/vehicle/boat\_simulator.py         |       47 |        0 |       10 |        0 |    100% |           |
| rover\_wizard/vehicle/boat\_telemetry.py         |       15 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/vehicle/demo.py                    |       13 |        0 |        4 |        0 |    100% |           |
| rover\_wizard/vehicle/link.py                    |      787 |       24 |      228 |       17 |     96% |544-\>548, 563, 575-576, 604-605, 633-634, 644-645, 692, 695, 752-\>exit, 774-777, 869-871, 991, 1057-\>1055, 1207, 1215, 1232, 1273, 1285, 1430, 1442 |
| rover\_wizard/vehicle/log\_transfer.py           |      209 |       11 |       74 |       13 |     90% |123-\>131, 127-129, 134, 217-218, 220-\>222, 225-227, 233-\>235, 242-\>244, 245, 246-\>248, 272, 276-\>280, 280-\>exit |
| rover\_wizard/vehicle/rc\_scaling.py             |       24 |        1 |        8 |        1 |     94% |        39 |
| rover\_wizard/vehicle/rover\_simulator.py        |      637 |        9 |      196 |       10 |     97% |729, 827, 829, 849-850, 874, 877-882, 1115-\>1118, 1120-\>1122, 1253-\>exit, 1415 |
| rover\_wizard/vehicle/sail\_simulator.py         |       97 |        0 |       32 |        1 |     99% | 119-\>121 |
| rover\_wizard/vehicle/sim\_logs.py               |       63 |        3 |       28 |        4 |     92% |49-50, 61-\>exit, 80, 97-\>exit |
| **TOTAL**                                        | **8135** |  **268** | **2160** |  **244** | **95%** |           |


## Setup coverage badge

Below are examples of the badges you can use in your main branch `README` file.

### Direct image

[![Coverage badge](https://raw.githubusercontent.com/heeplr/rover-wizard/python-coverage-comment-action-data/badge.svg)](https://htmlpreview.github.io/?https://github.com/heeplr/rover-wizard/blob/python-coverage-comment-action-data/htmlcov/index.html)

This is the one to use if your repository is private or if you don't want to customize anything.

### [Shields.io](https://shields.io) Json Endpoint

[![Coverage badge](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/heeplr/rover-wizard/python-coverage-comment-action-data/endpoint.json)](https://htmlpreview.github.io/?https://github.com/heeplr/rover-wizard/blob/python-coverage-comment-action-data/htmlcov/index.html)

Using this one will allow you to [customize](https://shields.io/endpoint) the look of your badge.
It won't work with private repositories. It won't be refreshed more than once per five minutes.

### [Shields.io](https://shields.io) Dynamic Badge

[![Coverage badge](https://img.shields.io/badge/dynamic/json?color=brightgreen&label=coverage&query=%24.message&url=https%3A%2F%2Fraw.githubusercontent.com%2Fheeplr%2Frover-wizard%2Fpython-coverage-comment-action-data%2Fendpoint.json)](https://htmlpreview.github.io/?https://github.com/heeplr/rover-wizard/blob/python-coverage-comment-action-data/htmlcov/index.html)

This one will always be the same color. It won't work for private repos. I'm not even sure why we included it.

## What is that?

This branch is part of the
[python-coverage-comment-action](https://github.com/marketplace/actions/python-coverage-comment)
GitHub Action. All the files in this branch are automatically generated and may be
overwritten at any moment.