# Isaac Teleop Publisher Setup

This page documents the Isaac Teleop / CloudXR bring-up for **G1 with a Thor backpack** up to the point where `GR00T-WholeBodyControl` can see `/xr_teleop/full_body` and `/xr_teleop/controller_data`.

After those topics are live, continue with this repo's ROS bridge, deploy, and optional data-collection commands by following [VR Teleop Setup](../getting_started/vr_teleop_setup.md), [VR Whole-Body Teleop](../tutorials/vr_wholebody_teleop.md), or [Data Collection](../tutorials/data_collection.md).

This page is a condensed, repo-specific version of the Isaac Teleop docs for:

- [Quick Start](https://nvidia.github.io/IsaacTeleop/main/getting_started/quick_start.html)
- [`examples/teleop_ros2` reference publisher](https://github.com/NVIDIA/IsaacTeleop/tree/main/examples/teleop_ros2)

```{admonition} Scope
:class: important
This workflow is currently documented and supported only for **G1 + Thor backpack**.
```

## Step 1: Prepare the Thor Host

Install the prerequisites on Thor:

```bash
sudo apt install -y build-essential curl git-lfs

ARCH=$(uname -m)
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-${ARCH}.sh"
bash "Miniforge3-Linux-${ARCH}.sh" -b -p "$HOME/miniforge3"
"$HOME/miniforge3/bin/conda" init
source ~/.bashrc

git lfs install
sudo usermod -aG docker $USER
newgrp docker
```

For Thor performance, enable the max power mode before teleoperation:

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

Optional thermal / over-current check:

```bash
cat /sys/class/hwmon/hwmon*/oc*_event_cnt
```

## Step 2: Clone and Build Isaac Teleop Components

Clone the Isaac Teleop repo and build the ROS publisher image plus camera streamer:

```bash
git clone --recurse-submodules git@github.com:NVIDIA/IsaacTeleop.git
cd IsaacTeleop

docker build -f examples/teleop_ros2/Dockerfile -t teleop_ros2_ref .

cd examples/camera_streamer
./camera_streamer.sh build
```

## Step 3: Launch the CloudXR Runtime via Docker

Start CloudXR before launching the ROS publisher:

```bash
cd IsaacTeleop
./scripts/run_cloudxr_via_docker.sh
```

The script sources `scripts/setup_cloudxr_env.sh`, prepares the CloudXR runtime variables, and brings up the CloudXR stack with Docker Compose.

Success indicators:

- Terminal output includes `CloudXR runtime: running`
- Terminal output includes `CloudXR WSS proxy: running`
- Logs appear under `~/.cloudxr/logs/`
- The shared CloudXR environment is exported for follow-on terminals through the runtime env file written by `scripts/setup_cloudxr_env.sh`

Keep this terminal running while you connect the XR client and start the publisher.

## Step 4: Connect the XR Client

- Open [Isaac Teleop Web Client](https://nvidia.github.io/IsaacTeleop/client/) in the headset browser
- Enter the IP address of the Thor host running CloudXR
- Accept the self-signed certificate at `https://<thor-ip>:48322`
- Return to the client page and click `Connect`

For quick validation, the same client URL can also be opened in a desktop browser.

If you prefer to run the WebXR client from source instead of using the hosted client, follow the CloudXR/WebXR build instructions linked from the [Isaac Teleop Quick Start](https://nvidia.github.io/IsaacTeleop/main/getting_started/quick_start.html).

```{important}
The ROS publisher will fail to acquire OpenXR until the XR client is connected. Complete this step before starting `teleop_ros2_ref`.
```

## Step 5: Launch the ROS Publisher

Once CloudXR is running and the XR client is connected, source the CloudXR helper script from the Isaac Teleop checkout and start the ROS publisher container:

```bash
cd IsaacTeleop
source scripts/setup_cloudxr_env.sh

docker run --rm --net=host --ipc=host \
    -e XR_RUNTIME_JSON \
    -e NV_CXR_RUNTIME_DIR \
    -e ROS_LOCALHOST_ONLY=1 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v $CXR_HOST_VOLUME_PATH:$CXR_HOST_VOLUME_PATH:ro \
    --name teleop_ros2_ref \
    teleop_ros2_ref \
    --ros-args -p mode:=full_body
```

## Step 7: Start Camera Streaming (Optional)

(Optional) Start camera streaming to view the camera feed in the headset.

```bash
cd IsaacTeleop
source scripts/setup_cloudxr_env.sh
cd examples/camera_streamer

./camera_streamer.sh build # if you haven't already
./camera_streamer.sh list-cameras
```

Run with local cameras streaming to the XR headset:

```bash
./camera_streamer.sh run --source local --mode xr
```

## Step 8: Validate the ROS Topics Before Starting This Repo

Before launching the `GR00T-WholeBodyControl` bridge on Thor, confirm that the publisher is already live:

```bash
docker exec -it teleop_ros2_ref /bin/bash
ros2 topic list
ros2 topic info /xr_teleop/full_body
ros2 topic info /xr_teleop/controller_data
```

You should see both `/xr_teleop/full_body` and `/xr_teleop/controller_data` in the active topic list.

```{admonition} Local Compatibility Contract
:class: note
This repo expects `std_msgs/msg/ByteMultiArray` carrying msgpack payloads on both topics. The local reader in `gear_sonic/utils/teleop/input_readers.py` decodes:

- full-body payloads with fields like `timestamp`, `joint_positions`, `joint_orientations`, and optional `is_active`
- controller payloads with trigger, squeeze, thumbstick, thumbstick click, and primary/secondary face-button state
```

Once those checks pass, return to this repo's `GR00T-WholeBodyControl` instructions on Thor:

- [VR Teleop Setup](../getting_started/vr_teleop_setup.md)
- [VR Whole-Body Teleop](../tutorials/vr_wholebody_teleop.md)
- [Data Collection](../tutorials/data_collection.md)

## Troubleshooting

### `RuntimeError: Failed to get OpenXR system: -35`

In this setup, that error usually means the XR client is not connected yet. Re-check:

- CloudXR runtime is still running
- The headset or web client is fully connected
- `source scripts/setup_cloudxr_env.sh` has been applied in the current shell before starting `teleop_ros2_ref`

### `/xr_teleop/*` topics are missing

Check the sequence again:

1. CloudXR runtime is running
2. XR client is connected
3. `teleop_ros2_ref` is running with `mode:=full_body`
4. You are checking the topics from the same ROS environment where the publisher is visible

### This repo cannot see the topics

Do not assume distributed ROS2 discovery is configured. This page uses `ROS_LOCALHOST_ONLY=1` on both the publisher and `GR00T-WholeBodyControl` bridge, so run both on the same Thor host and in the same local ROS environment where `ros2 topic list` already shows `/xr_teleop/full_body` and `/xr_teleop/controller_data`.
