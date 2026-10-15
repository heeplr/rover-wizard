# Repository Coverage

[Full report](https://htmlpreview.github.io/?https://github.com/heeplr/rover-wizard/blob/python-coverage-comment-action-data/htmlcov/index.html)

| Name                                       |    Stmts |     Miss |   Branch |   BrPart |   Cover |   Missing |
|------------------------------------------- | -------: | -------: | -------: | -------: | ------: | --------: |
| rover\_wizard/\_\_init\_\_.py              |       19 |        6 |        2 |        1 |     67% |     29-34 |
| rover\_wizard/\_\_main\_\_.py              |       36 |        9 |       10 |        3 |     70% |23-25, 52-53, 57-60, 64 |
| rover\_wizard/ardupilot\_defs.py           |       51 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/context.py                   |      289 |       21 |      112 |       10 |     91% |107, 126-127, 133, 151, 161, 193-\>195, 199-\>206, 259-260, 280, 287, 291, 385-392, 423-425 |
| rover\_wizard/flow.py                      |       27 |        2 |        6 |        2 |     88% |     49-50 |
| rover\_wizard/mavlink\_defs.py             |      100 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/model.py                     |       77 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/motion/\_\_init\_\_.py       |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/motion/analysis.py           |      204 |        7 |       50 |        8 |     94% |231, 284, 312-\>320, 364, 385, 418, 434, 437 |
| rover\_wizard/motion/driving.py            |      172 |       15 |       46 |        6 |     89% |64, 136-138, 229-230, 239-\>242, 280, 296-300, 307-308, 330-331 |
| rover\_wizard/motion/servo\_test.py        |      236 |        8 |       58 |        5 |     96% |112-113, 117-118, 126-127, 266-\>268, 393, 419 |
| rover\_wizard/param\_cli.py                |      126 |        5 |       40 |        2 |     96% |96, 121-123, 163 |
| rover\_wizard/session/\_\_init\_\_.py      |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/session/changes.py           |       24 |        2 |        2 |        0 |     92% |     36-37 |
| rover\_wizard/session/store.py             |       84 |        6 |       18 |        4 |     88% |26, 28, 76-80, 113-\>116 |
| rover\_wizard/steps/\_\_init\_\_.py        |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/accel.py               |       72 |        7 |       18 |        5 |     87% |49, 65-66, 95, 104, 127, 146 |
| rover\_wizard/steps/battery.py             |       52 |        5 |       14 |        4 |     86% |52-\>57, 55-\>57, 63-64, 75, 79, 105 |
| rover\_wizard/steps/beacon.py              |       16 |        1 |        4 |        2 |     85% |19-\>exit, 22 |
| rover\_wizard/steps/compass.py             |      297 |       18 |       96 |       21 |     90% |80-\>82, 85, 98, 118-119, 140, 147, 160, 186-\>197, 203, 250, 275, 281, 472, 480-\>482, 482-\>484, 487, 532, 539, 543, 567, 588 |
| rover\_wizard/steps/direction\_check.py    |       65 |        3 |       26 |        6 |     90% |101, 107, 122, 129-\>133, 135-\>144, 140-\>144 |
| rover\_wizard/steps/distance\_sensors.py   |      173 |       13 |       52 |        8 |     90% |189-196, 290, 297, 342-343, 360-\>350, 393, 401-402, 415-\>408, 421-422 |
| rover\_wizard/steps/ekf\_sources.py        |       61 |        2 |       10 |        2 |     94% |  101, 114 |
| rover\_wizard/steps/failsafe.py            |       20 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/final.py               |        4 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/gains.py               |       68 |        4 |       20 |        6 |     89% |75, 82, 93-\>103, 100-\>96, 126, 132 |
| rover\_wizard/steps/generator.py           |       15 |        1 |        4 |        2 |     84% |31-\>exit, 34 |
| rover\_wizard/steps/gimbal.py              |       75 |        8 |       16 |        5 |     86% |89-\>92, 101, 117-118, 142, 152-153, 167-168 |
| rover\_wizard/steps/gps.py                 |      116 |       14 |       34 |        6 |     84% |92-93, 111, 118-119, 125-133, 198, 213, 215-\>exit |
| rover\_wizard/steps/gripper.py             |       31 |        3 |        4 |        2 |     86% | 44-46, 58 |
| rover\_wizard/steps/log\_settings.py       |       12 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/motor\_output.py       |      108 |        4 |       32 |        5 |     94% |84, 125, 186-\>207, 189, 216 |
| rover\_wizard/steps/navigation.py          |       17 |       10 |        2 |        0 |     37% |     17-35 |
| rover\_wizard/steps/optical\_flow.py       |        5 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/orientation.py         |       61 |        3 |       14 |        4 |     91% |25-\>29, 32, 76, 92 |
| rover\_wizard/steps/osd.py                 |        7 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/predrive.py            |       76 |        2 |       20 |        6 |     92% |43, 61-\>70, 82-\>89, 89-\>91, 98-\>115, 116-\>123, 185 |
| rover\_wizard/steps/rc.py                  |      101 |        6 |       28 |        5 |     91% |112, 131-\>130, 147, 159-160, 163-164 |
| rover\_wizard/steps/relays.py              |       37 |        1 |        6 |        1 |     95% |        50 |
| rover\_wizard/steps/remote\_id.py          |       22 |        2 |        6 |        2 |     86% |    33, 37 |
| rover\_wizard/steps/rpm.py                 |       30 |        0 |        4 |        1 |     97% | 53-\>exit |
| rover\_wizard/steps/save.py                |       12 |        7 |        4 |        0 |     31% |     11-19 |
| rover\_wizard/steps/scripting.py           |       18 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/serial\_ports.py       |       20 |        0 |        4 |        0 |    100% |           |
| rover\_wizard/steps/servo\_outputs.py      |       43 |        2 |        8 |        3 |     90% |70-\>76, 74, 77 |
| rover\_wizard/steps/sprayer.py             |       16 |        2 |        4 |        1 |     85% |     24-26 |
| rover\_wizard/steps/steering\_output.py    |       18 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/temperature.py         |       11 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/top\_speed.py          |       31 |        1 |        8 |        3 |     90% |54, 62-\>66, 66-\>72 |
| rover\_wizard/steps/turning.py             |       51 |        4 |       10 |        5 |     85% |64, 71, 95, 102, 110-\>115 |
| rover\_wizard/steps/util/\_\_init\_\_.py   |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/util/actions.py        |       19 |        2 |        8 |        3 |     81% |13, 18-\>exit, 25 |
| rover\_wizard/steps/util/choices.py        |        3 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/util/compass\_slots.py |       27 |        2 |       10 |        2 |     89% |    40, 55 |
| rover\_wizard/steps/util/driving.py        |       15 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/util/hardware.py       |       60 |        1 |        6 |        1 |     97% |       128 |
| rover\_wizard/steps/util/outputs.py        |       80 |        1 |       36 |        2 |     97% |180-\>exit, 228 |
| rover\_wizard/steps/util/ports.py          |       40 |        1 |        8 |        1 |     96% |        62 |
| rover\_wizard/steps/util/probing.py        |       23 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/vehicle\_type.py       |       34 |        1 |       14 |        1 |     96% |        53 |
| rover\_wizard/steps/verify.py              |      106 |       20 |       26 |        6 |     77% |84, 116-148, 180, 191, 208-209, 217, 244-245 |
| rover\_wizard/steps/visual\_odometry.py    |        5 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/wheel\_encoders.py     |       57 |       10 |       18 |        5 |     75% |66, 73-78, 85, 95, 97, 99 |
| rover\_wizard/ui/\_\_init\_\_.py           |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/ui/connect\_screen.py        |       60 |       17 |        8 |        1 |     65% |54-\>exit, 59-72, 98-99, 103-104 |
| rover\_wizard/ui/main\_screen.py           |      453 |       39 |      162 |       28 |     88% |206-\>208, 209-\>211, 212, 259, 263-264, 356, 358-359, 445, 450-\>441, 454-\>441, 462, 468, 491, 493-494, 499-501, 503-\>exit, 507, 511-513, 517, 522, 534-\>exit, 543, 545, 547-\>exit, 570-571, 575-576, 592-595, 600, 617-618, 629, 643, 647-648, 662-\>exit |
| rover\_wizard/ui/markdown.py               |        4 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/ui/modals.py                 |      125 |       11 |        8 |        3 |     89% |38, 69-70, 74, 196-202, 217, 220-\>exit |
| rover\_wizard/ui/monitor.py                |       71 |        1 |       16 |        2 |     97% |72, 120-\>119 |
| rover\_wizard/ui/param\_docs.py            |       68 |        0 |       20 |        0 |    100% |           |
| rover\_wizard/ui/param\_screen.py          |      247 |       28 |       56 |       15 |     85% |57-58, 92-\>94, 94-\>99, 139-\>exit, 144, 151, 157-158, 164, 192-\>196, 202, 205, 244-247, 366-\>exit, 373-374, 380, 384-386, 391-392, 396-\>exit, 420-421, 429-431, 445, 453-454 |
| rover\_wizard/ui/step\_catalog.py          |       25 |        0 |        6 |        0 |    100% |           |
| rover\_wizard/ui/wizard\_app.py            |       44 |        5 |        4 |        1 |     83% | 65, 76-79 |
| rover\_wizard/util/\_\_init\_\_.py         |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/util/errors.py               |        3 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/util/rotation.py             |       59 |        3 |       18 |        3 |     92% |80, 106, 144 |
| rover\_wizard/util/timebase.py             |       12 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/util/values.py               |       22 |        2 |        8 |        2 |     87% |    30, 32 |
| rover\_wizard/vehicle/\_\_init\_\_.py      |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/vehicle/link.py              |      655 |       68 |      194 |       29 |     86% |445-\>449, 464, 476-477, 505-506, 510-\>484, 534-535, 542-\>exit, 545-546, 556, 582, 584, 598, 643-\>exit, 665-668, 709-\>708, 711, 737, 747-763, 766, 848-\>841, 855, 882, 904-\>901, 912-915, 928-\>924, 932, 943, 1031-1033, 1045, 1055, 1096, 1108, 1127-1136, 1141-1160, 1206, 1218 |
| rover\_wizard/vehicle/rc\_scaling.py       |       24 |        1 |        8 |        1 |     94% |        39 |
| rover\_wizard/vehicle/simulator.py         |      492 |       10 |      168 |        8 |     97% |597, 659, 661, 681-682, 706, 709-714, 796, 1041-\>exit, 1160 |
| **TOTAL**                                  | **6039** |  **427** | **1604** |  **260** | **90%** |           |


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