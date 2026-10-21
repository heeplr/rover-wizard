# Repository Coverage

[Full report](https://htmlpreview.github.io/?https://github.com/heeplr/rover-wizard/blob/python-coverage-comment-action-data/htmlcov/index.html)

| Name                                        |    Stmts |     Miss |   Branch |   BrPart |   Cover |   Missing |
|-------------------------------------------- | -------: | -------: | -------: | -------: | ------: | --------: |
| rover\_wizard/\_\_init\_\_.py               |       19 |        6 |        2 |        1 |     67% |     29-34 |
| rover\_wizard/\_\_main\_\_.py               |       38 |        1 |       12 |        1 |     96% |        69 |
| rover\_wizard/ardupilot\_defs.py            |       66 |       66 |        0 |        0 |      0% |     8-211 |
| rover\_wizard/boat\_defs.py                 |       33 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/context.py                    |      296 |        8 |      112 |        8 |     96% |108, 127-128, 134, 162, 206-\>208, 212-\>219, 293, 300, 304 |
| rover\_wizard/flow.py                       |       27 |        0 |        6 |        0 |    100% |           |
| rover\_wizard/mavlink\_defs.py              |      102 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/model.py                      |       85 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/motion/\_\_init\_\_.py        |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/motion/analysis.py            |      204 |        4 |       50 |        5 |     96% |231, 284, 312-\>320, 364, 385 |
| rover\_wizard/motion/driving.py             |      178 |        5 |       48 |        2 |     97% |66, 238-239, 248-\>251, 339-340 |
| rover\_wizard/motion/sail\_test.py          |       11 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/motion/servo\_test.py         |      255 |        8 |       66 |        5 |     96% |118-119, 123-124, 132-133, 272-\>274, 406, 441 |
| rover\_wizard/param\_cli.py                 |      126 |        5 |       40 |        2 |     96% |96, 121-123, 163 |
| rover\_wizard/rover\_defs.py                |       69 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/session/\_\_init\_\_.py       |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/session/changes.py            |       24 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/session/store.py              |       84 |        0 |       18 |        1 |     99% | 113-\>116 |
| rover\_wizard/steps/\_\_init\_\_.py         |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/accel.py                |       72 |        7 |       18 |        5 |     87% |49, 65-66, 95, 104, 127, 146 |
| rover\_wizard/steps/barometer.py            |       60 |        2 |       18 |        2 |     95% |   54, 101 |
| rover\_wizard/steps/battery.py              |       52 |        0 |       14 |        1 |     98% |   52-\>57 |
| rover\_wizard/steps/beacon.py               |       16 |        1 |        4 |        2 |     85% |19-\>exit, 22 |
| rover\_wizard/steps/boat\_loiter.py         |        6 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/compass.py              |      295 |       11 |       92 |       14 |     94% |84, 97, 146, 185-\>196, 202, 249, 274, 280, 471, 479-\>481, 481-\>483, 486, 571, 592 |
| rover\_wizard/steps/depth\_sounder.py       |       61 |        1 |       12 |        2 |     96% |111-\>exit, 114 |
| rover\_wizard/steps/direction\_check.py     |       66 |        3 |       26 |        6 |     90% |102, 108, 123, 130-\>134, 136-\>145, 141-\>145 |
| rover\_wizard/steps/distance\_sensors.py    |      173 |        9 |       52 |        8 |     92% |290, 297, 342-343, 360-\>350, 393, 401-402, 415-\>408, 421-422 |
| rover\_wizard/steps/ekf\_sources.py         |       61 |        2 |       10 |        2 |     94% |  101, 114 |
| rover\_wizard/steps/failsafe.py             |       20 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/final.py                |        4 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/gains.py                |       69 |        4 |       20 |        6 |     89% |76, 83, 94-\>104, 101-\>97, 127, 133 |
| rover\_wizard/steps/generator.py            |       15 |        1 |        4 |        2 |     84% |31-\>exit, 34 |
| rover\_wizard/steps/gimbal.py               |       75 |        8 |       16 |        5 |     86% |89-\>92, 101, 117-118, 142, 152-153, 167-168 |
| rover\_wizard/steps/gps.py                  |      117 |        2 |       34 |        3 |     97% |140, 225, 227-\>exit |
| rover\_wizard/steps/gripper.py              |       31 |        3 |        4 |        2 |     86% | 44-46, 58 |
| rover\_wizard/steps/log\_settings.py        |       12 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/motor\_output.py        |      112 |        5 |       34 |        6 |     92% |86, 127, 188-\>213, 191, 211, 222 |
| rover\_wizard/steps/navigation.py           |       17 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/optical\_flow.py        |        5 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/orientation.py          |       52 |        3 |       10 |        3 |     90% |25, 69, 85 |
| rover\_wizard/steps/osd.py                  |        7 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/predrive.py             |       76 |        2 |       20 |        6 |     92% |43, 61-\>70, 82-\>89, 89-\>91, 98-\>115, 116-\>123, 183 |
| rover\_wizard/steps/rc.py                   |      101 |        4 |       28 |        3 |     95% |131-\>130, 159-160, 163-164 |
| rover\_wizard/steps/relays.py               |       37 |        1 |        6 |        1 |     95% |        50 |
| rover\_wizard/steps/remote\_id.py           |       22 |        2 |        6 |        2 |     86% |    33, 37 |
| rover\_wizard/steps/rpm.py                  |       30 |        0 |        4 |        1 |     97% | 53-\>exit |
| rover\_wizard/steps/sails.py                |       52 |        2 |       16 |        3 |     93% |93, 105, 116-\>exit |
| rover\_wizard/steps/save.py                 |       12 |        0 |        4 |        0 |    100% |           |
| rover\_wizard/steps/scripting.py            |       18 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/serial\_ports.py        |       20 |        0 |        4 |        0 |    100% |           |
| rover\_wizard/steps/servo\_outputs.py       |       43 |        2 |        8 |        3 |     90% |70-\>76, 74, 77 |
| rover\_wizard/steps/sprayer.py              |       16 |        2 |        4 |        1 |     85% |     24-26 |
| rover\_wizard/steps/steering\_output.py     |       21 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/temperature.py          |       11 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/top\_speed.py           |       32 |        1 |        8 |        3 |     90% |55, 63-\>67, 67-\>73 |
| rover\_wizard/steps/torqeedo.py             |       52 |        2 |       12 |        3 |     92% |62, 69, 75-\>77 |
| rover\_wizard/steps/turning.py              |       52 |        4 |       10 |        5 |     85% |65, 72, 96, 103, 111-\>116 |
| rover\_wizard/steps/util/\_\_init\_\_.py    |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/util/actions.py         |       19 |        2 |        8 |        3 |     81% |13, 18-\>exit, 25 |
| rover\_wizard/steps/util/boat\_hardware.py  |       12 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/util/choices.py         |        3 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/util/compass\_slots.py  |       27 |        2 |       10 |        2 |     89% |    40, 55 |
| rover\_wizard/steps/util/driving.py         |       15 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/util/hardware.py        |       60 |       60 |        6 |        0 |      0% |     4-239 |
| rover\_wizard/steps/util/outputs.py         |       83 |        1 |       38 |        1 |     98% |       226 |
| rover\_wizard/steps/util/ports.py           |       40 |        1 |        8 |        1 |     96% |        62 |
| rover\_wizard/steps/util/probing.py         |       36 |        0 |        6 |        1 |     98% |   65-\>67 |
| rover\_wizard/steps/util/rover\_hardware.py |       78 |        0 |        8 |        0 |    100% |           |
| rover\_wizard/steps/vehicle\_type.py        |       41 |        1 |       14 |        1 |     96% |        76 |
| rover\_wizard/steps/verify.py               |      107 |        7 |       26 |        5 |     91% |85, 143-144, 181, 192, 245-246 |
| rover\_wizard/steps/visual\_odometry.py     |        5 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/wheel\_encoders.py      |       57 |        5 |       18 |        5 |     87% |66, 75, 85, 95, 97 |
| rover\_wizard/steps/wind\_vane.py           |      108 |        7 |       28 |        9 |     88% |116, 125, 129, 148, 151, 190, 205, 207-\>209, 209-\>211 |
| rover\_wizard/texts.py                      |       24 |        0 |        4 |        0 |    100% |           |
| rover\_wizard/ui/\_\_init\_\_.py            |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/ui/connect\_screen.py         |       60 |        0 |        8 |        1 |     99% | 62-\>exit |
| rover\_wizard/ui/main\_screen.py            |      466 |       20 |      162 |       19 |     93% |225, 282-283, 375, 377-378, 472-\>463, 476-\>463, 484, 490, 513, 521-523, 525-\>exit, 529, 533-\>exit, 556-\>exit, 569-\>exit, 614-\>616, 622, 642-644, 662, 676, 680-681, 695-\>exit |
| rover\_wizard/ui/markdown.py                |        4 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/ui/modals.py                  |      125 |        9 |        8 |        3 |     91% |38, 74, 196-202, 217, 220-\>exit |
| rover\_wizard/ui/monitor.py                 |       74 |        1 |       16 |        2 |     97% |72, 120-\>119 |
| rover\_wizard/ui/param\_docs.py             |       68 |        0 |       20 |        0 |    100% |           |
| rover\_wizard/ui/param\_screen.py           |      247 |       16 |       56 |        8 |     92% |57-58, 141-\>exit, 146, 166, 204, 247, 368-\>exit, 375-376, 398-\>exit, 422-423, 431-433, 447, 455-456 |
| rover\_wizard/ui/step\_catalog.py           |       19 |        0 |        6 |        0 |    100% |           |
| rover\_wizard/ui/wizard\_app.py             |       43 |        4 |        4 |        0 |     87% |     73-76 |
| rover\_wizard/util/\_\_init\_\_.py          |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/util/errors.py                |        3 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/util/rotation.py              |       59 |        3 |       18 |        3 |     92% |80, 106, 144 |
| rover\_wizard/util/timebase.py              |       12 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/util/values.py                |       22 |        2 |        8 |        2 |     87% |    30, 32 |
| rover\_wizard/vehicle/\_\_init\_\_.py       |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/vehicle/boat\_simulator.py    |       47 |        0 |       10 |        0 |    100% |           |
| rover\_wizard/vehicle/boat\_telemetry.py    |       15 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/vehicle/demo.py               |       11 |        2 |        4 |        1 |     80% |     17-19 |
| rover\_wizard/vehicle/link.py               |      671 |       23 |      200 |       18 |     95% |450-\>454, 469, 481-482, 510-511, 515-\>489, 539-540, 547-\>exit, 550-551, 594, 651-\>exit, 673-676, 768-770, 890, 922-\>920, 1053, 1061, 1078, 1119, 1131, 1234, 1246 |
| rover\_wizard/vehicle/rc\_scaling.py        |       24 |        1 |        8 |        1 |     94% |        39 |
| rover\_wizard/vehicle/rover\_simulator.py   |      525 |        9 |      178 |        8 |     97% |647, 725, 727, 747-748, 772, 775-780, 1111-\>exit, 1246 |
| rover\_wizard/vehicle/sail\_simulator.py    |       92 |        3 |       30 |        4 |     94% |117-\>119, 140, 150, 190 |
| rover\_wizard/vehicle/simulator.py          |      515 |      515 |      178 |        0 |      0% |    9-1372 |
| **TOTAL**                                   | **7427** |  **881** | **1954** |  **223** | **86%** |           |


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