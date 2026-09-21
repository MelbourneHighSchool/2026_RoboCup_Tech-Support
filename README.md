# Tech Support 2026

We are Tech Support, a robotics team from Melbourne High School (MHS) competing in the RoboCup Junior Australia (RCJA) Soccer Open competition. This repo contains all the code used for our robots.

This repo was created to share our code to provide inspiration to other teams. However, as stated in the [RCJA General Rules](https://www.robocupjunior.org.au/wp-content/uploads/2026/02/2026-RCJA-General-Rules.pdf) in Rule 4.3.2:

> Teams may not directly use designs and programs that has been passed down to them by the teams before them

## Code Structure

`tests/` contains various test files to test various parts of our bot

`lib/` contains code that directly interfaces with the hardware on the Pi

`STM32/` contains the firmware for the STM32 on our PCB

`calibration/` contains motor driver calibration (`calibration/motors.py`) and ball-distance calibration (`calibration/ball_distance.py`).

`training/` contains all the code used to train our models on my hardware. I do not recommend using this code as it was only used to train a very specific model with a very specific setup on my specific computer.

`legacy/` contains old code which is no longer used, such as the python movement controller (this was ported to a C++ Hardware Controller in `lib/`)

The below files are the most important, and are annotated extensively:

`config.txt` includes the settings that vary between bots. An example can be found in `example_config.txt`

`simulate.py` can be used to test controllers or replay recorded games.

`main.py` is the file that is run during gameplay, it uses functions from all the files below

`defence.py` includes the controllers for the goalie and a mode called defence, which is a simple mode that just gets the ball and pushes it forwards.

`striker.py` includes the controller for the striker

The models themselves are stored in `open-soccer-detect-n_hailo_model` and `open-soccer-detect-s_hailo_model`. These are trained models based on YOLO26n and YOLO26s respectively, and quantised to be run on a Raspberry Pi AI HAT.

## AI Declaration

Generative AI tools such as GitHub Copilot, Codex and Cursor were used to assist in writing the code in this repo. We believe in using AI to enhance human development, not replace it, so in accordance with the MHS Robotics Club Generative AI policy, all essential code (such as `main.py`, `defence.py`, etc) has been either written or extensively verified by our team. However, test code (such as the calibration dashboard) and code used for model training was mostly written by AI, as it is not used during gameplay.

## Contact

If you would like to contact us please use the following contact details:

Arhan - Email: [arhan@arhan.tech](mailto:arhan@arhan.tech) Discord: @arbusam

Kanishk - Discord: @kansar_1
## Calibration dashboard

Run `.venv/bin/python calibration_dashboard.py` on the Pi and open `http://<pi-ip>:8080` from another device on the LAN. Includes live ball/bot detections, distance and motor calibration, goal masks/HSV tuning, pixel picking, and localisation/target-driving diagnostics. See [the run guide and Pi acceptance checks](calibration/DASHBOARD.md).
