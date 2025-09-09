# alphabot_swarm
Code to deploy on Alphabots to test the STTR swarm control algorithms. Initial work performed by RI for STTR phase 2E.

Requirements

Hardware
1. AlphaBot2-Pi
2. Decawave MDEK1001 (ultra-wideband sensors)
3. SparkFun ICM-20948 (IMU)
4. CP2112 (I2C-USB Converter)

Setup ultra-wideband sensors
1. Download the Decawave Software Package (for DWM1001)
2. Flash the ultra-wideband sensors with J-Flash lite
3. Use the Decawave app (on Android) to set sensors as anchors or tags
The full guide is here: https://medium.com/@newforestberlin/precise-realtime-indoor-localization-with-raspberry-pi-and-ultra-wideband-technology-decawave-191e4e2daa8c
