# asr_sdm_video_enhancement_ml

ROS 2 Jazzy C++ node for FiveA+ underwater image enhancement with ONNX Runtime CPU inference.

The model architecture is adapted from UIE_Benckmark's MIT-licensed FiveA+ implementation:
https://github.com/ddz16/UIE_Benckmark

## Layout

- `src/`, `include/`: C++ ROS 2 node that loads the ONNX model and runs inference.

## Export

The `.pth` to `.onnx` conversion tool is kept outside this ROS package:

```bash
python3 /home/cortin/asr_sdm_robo/src/asr_sdm_tools/pth_to_onnx/export_five_aplus_onnx.py \
  --weights /home/cortin/Desktop/FIVE_APLUS_epoch97.pth \
  --output /home/cortin/ros2_ws/src/asr_sdm_video_enhancement_ml/models/five_aplus_epoch97.onnx
```

The checkpoint includes training-only `per_loss.*` weights. The export tool filters those keys before loading the network.

## Build

```bash
source /opt/ros/jazzy/setup.bash
export ONNXRUNTIME_ROOT=/home/cortin/.local/onnxruntime/current
cd /home/cortin/ros2_ws
colcon build --packages-select asr_sdm_video_enhancement_ml
```

## Run

```bash
source /opt/ros/jazzy/setup.bash
source /home/cortin/ros2_ws/install/setup.bash
ros2 launch asr_sdm_video_enhancement_ml asr_sdm_video_enhancement_ml.launch.py
```

Defaults:

- subscribes: `/camera/camera/color/image_raw`
- publishes: `/asr_sdm_video_enhancement_ml/image`
- input encodings: `bgr8`, `rgb8`
- minimum input size: `128x128`
