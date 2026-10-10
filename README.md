# TEACar ROS2

## Setup
1. Clone the repo
2. Install ros2 humble or newer version
3. Build the workspace

## Usage
### Drive the car
```bash
ros2 launch  drive.launch.py
```

### Use autopilot
1. Put the model files (e.g. `best.onnx` and `best.engine`) in `src/teacar_bringup/models/` and rebuild, or pass `model_file:=/path/to/model.onnx`
2. Run
```bash
ros2 launch teacar_bringup autopilot.launch.py
```
The neural network controls the steer, the throttle is controlled by the joystick.
The backend (`tensorrt` or `tensorflow`), tensor names and image roi are set in `config/autopilot_config.yaml`.

### Run an inferencer backend without ROS
```bash
ros2 run inferencer inferencer_cli tensorrt /path/to/model.onnx image steer [image.png 0 80 224 144]
```
