# Autonomously Driving Vehicle

## Overview

This repository contains the ROS2 workspace and associated projects developed as part of the "Robotik" module during the Summer Semester 2024. The primary objective is to design and implement a 1:10 scale autonomous vehicle capable of navigating unfamiliar tracks using a combination of cameras and various sensors. The system integrates multiple Arduino boards and a Raspberry Pi 4 as the main control unit, with inter-component communication facilitated through ROS2 on an Ubuntu-based operating system.

## Key Objectives

1. **Hardware Integration**

   - Utilize a Raspberry Pi 4 as the central processing unit.
   - Incorporate multiple Arduino boards for sensor data acquisition and actuator control.
   - Equip the vehicle with cameras and various sensors for environmental perception.

2. **Software Development**

   - Develop ROS2 nodes for sensor data processing, decision-making, and control algorithms.
   - Implement communication protocols between the Raspberry Pi and Arduino boards using ROS2 publisher and subscriber.

3. **Autonomous Navigation**

   - Implement algorithms for path planning and obstacle avoidance.
   - Develop computer vision techniques for track detection and following.

4. **System Integration and Testing**

   - Integrate hardware and software components into a cohesive system.
   - Test and validate the autonomous vehicle on various track configurations to assess performance and reliability.

## Core Features

1. **Sensor Fusion**

   - Combine data from multiple sensors to create an accurate representation of the vehicle's environment.

2. **Real-Time Processing**

   - Process sensor data and execute control algorithms in real-time to enable responsive navigation.

3. **Modular ROS2 Architecture**

   - Utilize a modular design with ROS2 nodes to ensure scalability and ease of maintenance.

4. **Camera-Based Track Detection**

   - Implement computer vision algorithms to detect and follow the track using camera input.

5. **Obstacle Detection and Avoidance**

   - Develop mechanisms to detect obstacles and adjust the vehicle's path accordingly.

## Technologies Used

- **Programming Languages**: Python, C++
- **Frameworks**: ROS2 (Robot Operating System 2)
- **Hardware**: Raspberry Pi 4, Arduino boards, cameras, various sensors
- **Operating System**: Ubuntu
- **Version Control**: Git & GitHub

## Getting Started

### Prerequisites

- Raspberry Pi 4 with Ubuntu installed
- Arduino boards
- ROS2 Foxy Fitzroy installed on the Raspberry Pi
- Cameras and sensors as specified in the hardware setup

### Installation

1. Clone the repository to your Raspberry Pi:

   ```bash
   git clone https://github.com/aakiev/Robotics-AutonomousDriving.git
   ```

2. Navigate to the workspace directory:

   ```bash
   cd WS_AlMa
   ```

3. Build the ROS2 workspace:

   ```bash
   colcon build
   ```

4. Source the setup file:

   ```bash
   source install/setup.bash
   ```

5. Upload the Arduino code to the respective boards using the Arduino IDE.

## Usage

1. **Launch ROS2 Nodes**:

   - Start the necessary ROS2 nodes for sensor data processing and control.

2. **Initialize the Vehicle**:

   - Ensure all hardware components are properly connected and powered.

3. **Start Autonomous Mode**:

   - Execute the main launch file to begin autonomous navigation.

4. **Monitor Performance**:

   - Use ROS2 tools to monitor topics and node statuses during operation.

## Future Enhancements

- Implement advanced path planning algorithms for more complex track layouts.
- Integrate additional sensors for improved environmental perception.
- Enhance computer vision capabilities for better track detection under varying lighting conditions.
- Optimize system performance for faster processing and response times.
