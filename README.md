# Handeye Calibration Tool



## Introduction

This tool is used to calibrate the handeye transformation between a robot and a camera.

The HMI is developed based on:
1. [PyQt5](https://www.riverbankcomputing.com/software/pyqt/)
2. [PyQt5-Fluent-Widget](https://github.com/zhiyiYo/PyQt-Fluent-Widgets).

## Installation
```
pip install -r requirements.txt
```

## Usage
For users, you can run the following command to start the tool:
```
python main.py
```
The calibration data will be saved as `calibration_result.csv` in the root directory.

## Configuration
You can configure the calibration parameters in the `.env` file.

**Chessboard_x_number**: the number of corners in the x direction of the chessboard.

**Chessboard_y_number**: the number of corners in the y direction of the chessboard.

**Chessboard_cell_size**: the size of the cell in the chessboard (mm).

**Translation_delta**: the translation delta for the robot to move (m).

**Angle_delta**: the rotational delta for the robot to move (degree).

**Calibration_sample**: the number of run for calibration.

**Robot_IP**: the IP address of the robot.

## Documentation
You can find the introduction video of this tool [here](<https://drive.google.com/file/d/19UvhzqAHppgQly6SWCmIvcy-RaeukK_O/view>)👈.

## Support Robot and Camera
- Robot: 
  - Universal Robot e-series
- Camera: 
  - Intel Realsense D-series

## Future Development Plan
- [x] Add robot type selection (only for Universal Robot now)
- [ ] Optimize the interface to support more robot and camera brands
  - [ ] Doosan
  - [ ] uFactory
  - [ ] Hikivision
- [ ] Add more calibration methods
