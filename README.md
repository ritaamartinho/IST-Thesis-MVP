# IST-Thesis-MVP
This repository is part of my MSc thesis in Electrical and Computer Engineering "IIoT for Lean Production Systems" at Instituto Superior Técnico. Here, the code of each device of the Minimum Viable Product developed (MVP) is available open source for anyone interested in using it or improving it.

The goal of this work was to develop a set of modular non-intrusive devices with the ability to acquire data on the production system. The MVP developed is composed by 4 devices:
- **RGB Node:** Developed to monitor machines states by observing light indicators of the machine. This device indicates the color (Red, Green, Blue or Off) of the light the sensor is pointed to. It uses LoRa and runs on Batteries.
- **ToF Node:** Developed for person detection (in desks and programming pannels) by identifying if there's someone in front of the device. It can also be used for object detection. This device indicates the distance from the sensor to the first object in front of it. Uses LoRa and runs on Batteries.
- **Vibration & Noise Node:** Developed to monitor machines states by monitoring its structure vibration and the noise produced. This device sends vibration and noise data for a deeper post analysis. Uses WiFi and needs to be plugged.
- **BLE Receiver (From the BLE System):** Together with BLE beacons composes the BLE System developed. This system provides information on the presence of employees (that carry the beacons) in monitored locations (where these BLE Receivers are placed). This device uses BLE to read the beacons signals, it uses WiFi to send the data to the cloud and needs to be plugged.

The data collected by these devices is sent to the ThingsBoard platform for data visualization. For a deeper analysis, a python script that downloads the data from the ThingsBoard platform throught its REST API was also made available.

To help users get started with the MVP, a short guide was developed and published in this repository.

More information on the development of these devices can be found in Section 3 of the Thesis document (also present in this repository).
