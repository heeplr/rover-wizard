# Repository Coverage

[Full report](https://htmlpreview.github.io/?https://github.com/heeplr/rover-wizard/blob/python-coverage-comment-action-data/htmlcov/index.html)

| Name                                       |    Stmts |     Miss |   Branch |   BrPart |   Cover |   Missing |
|------------------------------------------- | -------: | -------: | -------: | -------: | ------: | --------: |
| rover\_wizard/\_\_init\_\_.py              |       19 |        6 |        2 |        1 |     67% |     29-34 |
| rover\_wizard/\_\_main\_\_.py              |       36 |        1 |       10 |        1 |     96% |        66 |
| rover\_wizard/ardupilot\_defs.py           |       66 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/context.py                   |      289 |        8 |      112 |        8 |     96% |107, 126-127, 133, 161, 193-\>195, 199-\>206, 280, 287, 291 |
| rover\_wizard/flow.py                      |       27 |        1 |        6 |        1 |     94% |        49 |
| rover\_wizard/mavlink\_defs.py             |      102 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/model.py                     |       77 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/motion/\_\_init\_\_.py       |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/motion/analysis.py           |      204 |        4 |       50 |        5 |     96% |231, 284, 312-\>320, 364, 385 |
| rover\_wizard/motion/driving.py            |      178 |        5 |       48 |        2 |     97% |66, 238-239, 248-\>251, 339-340 |
| rover\_wizard/motion/servo\_test.py        |      242 |        8 |       60 |        5 |     96% |116-117, 121-122, 130-131, 270-\>272, 397, 431 |
| rover\_wizard/param\_cli.py                |      126 |        5 |       40 |        2 |     96% |96, 121-123, 163 |
| rover\_wizard/session/\_\_init\_\_.py      |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/session/changes.py           |       24 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/session/store.py             |       84 |        0 |       18 |        1 |     99% | 113-\>116 |
| rover\_wizard/steps/\_\_init\_\_.py        |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/accel.py               |       72 |        7 |       18 |        5 |     87% |49, 65-66, 95, 104, 127, 146 |
| rover\_wizard/steps/barometer.py           |       60 |        2 |       18 |        2 |     95% |   54, 101 |
| rover\_wizard/steps/battery.py             |       52 |        0 |       14 |        1 |     98% |   52-\>57 |
| rover\_wizard/steps/beacon.py              |       16 |        1 |        4 |        2 |     85% |19-\>exit, 22 |
| rover\_wizard/steps/compass.py             |      293 |       11 |       92 |       14 |     94% |84, 97, 146, 185-\>196, 202, 249, 274, 280, 471, 479-\>481, 481-\>483, 486, 566, 587 |
| rover\_wizard/steps/direction\_check.py    |       65 |        3 |       26 |        6 |     90% |101, 107, 122, 129-\>133, 135-\>144, 140-\>144 |
| rover\_wizard/steps/distance\_sensors.py   |      173 |        9 |       52 |        8 |     92% |290, 297, 342-343, 360-\>350, 393, 401-402, 415-\>408, 421-422 |
| rover\_wizard/steps/ekf\_sources.py        |       61 |        2 |       10 |        2 |     94% |  101, 114 |
| rover\_wizard/steps/failsafe.py            |       20 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/final.py               |        4 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/gains.py               |       68 |        4 |       20 |        6 |     89% |75, 82, 93-\>103, 100-\>96, 126, 132 |
| rover\_wizard/steps/generator.py           |       15 |        1 |        4 |        2 |     84% |31-\>exit, 34 |
| rover\_wizard/steps/gimbal.py              |       75 |        8 |       16 |        5 |     86% |89-\>92, 101, 117-118, 142, 152-153, 167-168 |
| rover\_wizard/steps/gps.py                 |      116 |        2 |       34 |        3 |     97% |128, 213, 215-\>exit |
| rover\_wizard/steps/gripper.py             |       31 |        3 |        4 |        2 |     86% | 44-46, 58 |
| rover\_wizard/steps/log\_settings.py       |       12 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/motor\_output.py       |      111 |        5 |       34 |        6 |     92% |85, 126, 187-\>212, 190, 210, 221 |
| rover\_wizard/steps/navigation.py          |       17 |        0 |        2 |        0 |    100% |           |
| rover\_wizard/steps/optical\_flow.py       |        5 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/orientation.py         |       52 |        3 |       10 |        3 |     90% |25, 69, 85 |
| rover\_wizard/steps/osd.py                 |        7 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/predrive.py            |       76 |        2 |       20 |        6 |     92% |43, 61-\>70, 82-\>89, 89-\>91, 98-\>115, 116-\>123, 185 |
| rover\_wizard/steps/rc.py                  |      101 |        4 |       28 |        3 |     95% |131-\>130, 159-160, 163-164 |
| rover\_wizard/steps/relays.py              |       37 |        1 |        6 |        1 |     95% |        50 |
| rover\_wizard/steps/remote\_id.py          |       22 |        2 |        6 |        2 |     86% |    33, 37 |
| rover\_wizard/steps/rpm.py                 |       30 |        0 |        4 |        1 |     97% | 53-\>exit |
| rover\_wizard/steps/save.py                |       12 |        0 |        4 |        0 |    100% |           |
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
| rover\_wizard/steps/util/hardware.py       |       60 |        0 |        6 |        0 |    100% |           |
| rover\_wizard/steps/util/outputs.py        |       80 |        1 |       36 |        2 |     97% |180-\>exit, 228 |
| rover\_wizard/steps/util/ports.py          |       40 |        1 |        8 |        1 |     96% |        62 |
| rover\_wizard/steps/util/probing.py        |       36 |        0 |        6 |        1 |     98% |   65-\>67 |
| rover\_wizard/steps/vehicle\_type.py       |       34 |        1 |       14 |        1 |     96% |        53 |
| rover\_wizard/steps/verify.py              |      106 |        7 |       26 |        5 |     91% |84, 142-143, 180, 191, 244-245 |
| rover\_wizard/steps/visual\_odometry.py    |        5 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/steps/wheel\_encoders.py     |       57 |        5 |       18 |        5 |     87% |66, 75, 85, 95, 97 |
| rover\_wizard/ui/\_\_init\_\_.py           |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/ui/connect\_screen.py        |       60 |        0 |        8 |        1 |     99% | 62-\>exit |
| rover\_wizard/ui/main\_screen.py           |      458 |       20 |      160 |       20 |     93% |224, 277, 281-282, 374, 376-377, 463, 468-\>459, 472-\>459, 480, 486, 509, 517-519, 521-\>exit, 525, 529-\>exit, 552-\>exit, 565-\>exit, 610-\>612, 618, 647, 661, 665-666, 680-\>exit |
| rover\_wizard/ui/markdown.py               |        4 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/ui/modals.py                 |      125 |        9 |        8 |        3 |     91% |38, 74, 196-202, 217, 220-\>exit |
| rover\_wizard/ui/monitor.py                |       74 |        1 |       16 |        2 |     97% |72, 120-\>119 |
| rover\_wizard/ui/param\_docs.py            |       68 |        0 |       20 |        0 |    100% |           |
| rover\_wizard/ui/param\_screen.py          |      247 |       16 |       56 |        8 |     92% |57-58, 141-\>exit, 146, 166, 204, 247, 368-\>exit, 375-376, 398-\>exit, 422-423, 431-433, 447, 455-456 |
| rover\_wizard/ui/step\_catalog.py          |       25 |        0 |        6 |        0 |    100% |           |
| rover\_wizard/ui/wizard\_app.py            |       44 |        4 |        4 |        0 |     88% |     76-79 |
| rover\_wizard/util/\_\_init\_\_.py         |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/util/errors.py               |        3 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/util/rotation.py             |       59 |        3 |       18 |        3 |     92% |80, 106, 144 |
| rover\_wizard/util/timebase.py             |       12 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/util/values.py               |       22 |        2 |        8 |        2 |     87% |    30, 32 |
| rover\_wizard/vehicle/\_\_init\_\_.py      |        0 |        0 |        0 |        0 |    100% |           |
| rover\_wizard/vehicle/link.py              |      667 |       22 |      198 |       17 |     95% |450-\>454, 469, 481-482, 510-511, 515-\>489, 539-540, 547-\>exit, 550-551, 648-\>exit, 670-673, 765-767, 887, 919-\>917, 1050, 1058, 1075, 1116, 1128, 1226, 1238 |
| rover\_wizard/vehicle/rc\_scaling.py       |       24 |        1 |        8 |        1 |     94% |        39 |
| rover\_wizard/vehicle/simulator.py         |      515 |        9 |      178 |        8 |     97% |640, 718, 720, 740-741, 765, 768-773, 1100-\>exit, 1235 |
| **TOTAL**                                  | **6174** |  **223** | **1636** |  **203** | **94%** |           |


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