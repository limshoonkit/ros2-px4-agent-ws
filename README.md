# ros2-px4-agent-ws


https://github.com/user-attachments/assets/e0ba7c40-c86a-4c45-ab21-ebf99011a593



https://github.com/user-attachments/assets/069eb3d0-76cd-48bf-947b-88494991641d


## Get started

1. Get [isaacsim](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html) and the asset packs to populate your scene.
2. Get the [px4 extension plugin](https://github.com/limshoonkit/uosm.isaac.px4_bridge)
3. Get dependencies for [ros-agents](src/ros-agents/README.md)

```
git clone --recursive https://github.com/limshoonkit/ros2-agent-ws.git
# git submodule update --init --recursive
```

## To build
```
colcon build --packages-select px4_msgs
source install/setup.bash
colcon build
```

## To run
In one terminal, run the following. Wait awhile for the models to be downloaded. 
```
source install/setup.bash
python src/ros-agents/px4_examples/demo_search_and_approach_agent.py
```

You can check with `ollama list`.

![Ollama list](media/ollama.png)

In another terminal, run the following to start the ros2 px4 control loop. 
Make sure the px4_msgs are populated
```
source install/setup.bash
ros2 launch px4_agent_control px4_agent_search_approach.launch.py
```

You should see the following node graph in rqt

![Node graph](media/rqt.png)

## Important Considerations

### 1. Drone Drift and PX4 Control Limitations
In some cases, the drone may drift excessively, and PX4’s control system may not compensate quickly enough. Consider reducing the sensor noise parameter in the PX4 OmniGraph inside Isaac Sim to improve stability.

### 2. Network Latency and Image Lag
Network delays can cause image transmission to the VLM to lag, potentially leading to inaccurate responses. To mitigate this, consider synchronizing clocks using Network Time Protocol (NTP).

### 3. Low Scene and Image Publishing FPS
If you're experiencing low frame rates for scene rendering or image publishing, try disabling colliders in the scene. The VLM only requires the visual mesh, so colliders are unnecessary for its operation.

### 4. Strict Goal Threshold
The goal threshold may be too tight for larger objects. In such cases, try setting the goal position at the edge of the object rather than its center of mass or adjust the threshold accordingly.
