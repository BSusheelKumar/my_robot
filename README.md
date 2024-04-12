# Autonomous Firefighting Robot

This project implements an intelligent firefighting robot that leverages an Internet of Things (IoT) network for early fire detection and response. The system prioritizes real-time performance and efficient resource utilization on the Raspberry Pi platform.

## Key Features:

- **Scalable Fire Detection:** Individual room fire detectors send signals via NodeMCU microcontrollers to a central IoT server, triggering robot deployment.
- **ROS 2 Powered Navigation:** The Raspberry Pi processes sensor data using ROS 2 for robust communication and control. LiDAR technology enables safe robot navigation within the burning environment.
- **Real-time Fire Detection (Optimized for Raspberry Pi):**
    - *OpenCV Haarcascade Classifier:* Due to the processing limitations of the Raspberry Pi for complex models like YOLO, this project employs the OpenCV Haarcascade classifier for real-time fire detection with minimal latency.

## Challenges and Solutions:

- **Raspberry Pi Limitations:** Initial attempts using YOLO and other machine learning models proved computationally expensive for the Raspberry Pi. The Haarcascade classifier offered a more lightweight and real-time solution.
- **False Positives:** Haarcascade classifiers can be sensitive to bright sunlight. To mitigate this, the robot employs a flame sensor for close-range confirmation, but this sensor can also be triggered by sunlight.

### Addressing Challenges:

- **Balancing Accuracy and Efficiency:** The project prioritizes real-time performance on the Raspberry Pi by utilizing the Haarcascade classifier. Future iterations could explore cloud-based processing for more advanced fire detection algorithms.
- **Mitigating False Positives:** A combination of fire detection methods (Haarcascade classifier and flame sensor) is used to reduce false positives, but further refinement might involve sensor fusion or environmental filtering techniques.
- **Distance Estimation (Without Depth Camera):** Given the absence of a depth camera, the flame sensor offers a basic solution for close-range stopping near the fire. More sophisticated techniques for distance estimation could be explored in future iterations.

## Benefits:

- **Early Fire Response:** The IoT network enables early fire detection and rapid robot deployment.
- **Safe Navigation:** LiDAR technology ensures safe robot movement in hazardous environments.
- **Real-time Fire Detection (Optimized for Raspberry Pi):** The Haarcascade classifier provides a balance between accuracy and computational efficiency.
- **Modular Design:** ROS 2 facilitates independent processing units for efficient robot operation.

## Demonstrated Expertise:

- Robotics
- ROS 2
- Sensor Integration (LiDAR, Camera, Flame Sensor)
- IoT Communication
- Firefighting Automation
- Real-time Image Processing with OpenCV (Haarcascade Classifier)
- Balancing Resource Constraints with Performance

## Future Enhancements:

- Explore cloud-based processing for more advanced fire detection algorithms.
- Investigate techniques for mitigating false positives due to sunlight.

