# alphabot_swarm

Code to deploy on AlphaBots to test the STTR swarm control algorithms.  
Initial work performed by **RI** for STTR Phase 2E.

---

## Requirements

### Hardware
- 4 × AlphaBot2-Pi  
- 7 × Decawave MDEK1001 (UWB sensors)  
- 4 × SparkFun ICM-20948 (IMU)  
- 4 × CP2112 (I2C–USB Converters)  

---

## Setup

### Ultra-Wideband Sensors
1. Download the [Decawave Software Package](https://www.qorvo.com/products/d/da008423).  
2. Flash the UWB sensors using **J-Flash Lite**.  
3. Use the **Decawave Android app** to configure sensors as anchors or tags.  

📖 Full guide: [Precise Real-Time Indoor Localization with Raspberry Pi and UWB](https://medium.com/@newforestberlin/precise-realtime-indoor-localization-with-raspberry-pi-and-ultra-wideband-technology-decawave-191e4e2daa8c)  

---

### Setup Swarm Controller (for programs in /commands)
1. Compile: make swarm_child.cpp and swarm_controller
2. Run ./swarm_controller on all robots 
3. Run ./swarm_child on all robots
Note: All robots must be setup starting at 0 degrees (facing +x-direction) to correctly calculate angle 

### Programs:
1. ssh_keyboard_drive.py: Control robot with WASD
2. imu.py: Reads orientation data using IMU 
3. uwb.py: Reads x,y position data using UWB sensors
4. positions_udp_main.py: Receives x,y position commands via UDP
5. vector_udp_main.py: Receives x,y vector commands via UDP

### Known Limitations:
1. IMU-based angle estimation drifts over time
2. UWB sensors may ocassionally require a restart (unplug/replug)
3. If UWB sensors stop receiving telemetry: reinitialize by running /position_estimation/uwb.py
