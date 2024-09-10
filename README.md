# Self-driving-Car
# **Self-Driving Car for National Robotics Competition**

## **Project Description**
This project aims to build a fully autonomous self-driving car for the **Future Engineers** category of the **National Robotics Competition**. The car is designed to complete three key challenges: 
1. **Time Attack Race**: The car will drive autonomously on a closed track to achieve the fastest lap times.
2. **Obstacle Challenge**: The car will detect and avoid randomly placed obstacles while navigating the track.
3. **Traffic Sign Challenge**: The car will follow green and red traffic signs, determining its lane position based on the signs and changing directions when a red sign is detected.

The vehicle is expected to navigate these challenges using a combination of sensors, motors, and intelligent algorithms that allow for precise control, obstacle avoidance, and traffic sign detection. The end goal is to complete all challenges and perform a parallel parking maneuver successfully.

---

## **Hardware Components**

### **1. Motors & Motor Driver**
- **DC Motors (2x)**: These are used for driving the car forward and backward. They provide the necessary torque to propel the vehicle. The motors are connected to the **rear wheels** of the vehicle and controlled by a motor driver.
- **Servo Motor (1x)**: This controls the steering system, allowing the vehicle to turn left or right. It is linked to the front wheels for precise direction control.
- **L298N Motor Driver (1x)**: The motor driver controls the speed and direction of the DC motors. It allows for differential control of the two motors while maintaining their synchronized movement. This is essential for both straight driving and turning during obstacle detection and lane changes.

### **2. Sensors**
- **Ultrasonic Sensor (HC-SR04)**: Used for obstacle detection. It measures the distance to an object by sending out sound waves and calculating the time taken for the echo to return. This data is used to avoid collisions.
- **Color Sensor (TCS3200)**: The color sensor detects green and red traffic signs placed on the track. Based on the color detected, the car decides to either continue in the current lane (green) or turn around (red).
- **IR Sensors (2x)**: Placed on the front of the car for line tracking. These sensors help the vehicle stay within its designated lane by detecting the boundaries of the track.
- **IMU Sensor (MPU6050)**: This sensor measures the car's orientation and acceleration. It helps maintain balance and ensures the car remains stable during turns and quick maneuvers.

### **3. Power Source**
- **LiPo Battery (7.4V)**: This lightweight battery powers the entire vehicle. It provides the necessary voltage to run both the motors and the sensors.

### **4. Controller**
- **Arduino Uno**: The main microcontroller that coordinates all the components. It processes the data from the sensors and controls the motors and servo for movement and steering.

---

## **Software**

### **1. Libraries Used**
- **Servo.h**: Used to control the steering servo motor.
- **NewPing.h**: Handles the ultrasonic sensor for distance measurement.
- **TCS3200.h**: A custom library to interface with the color sensor for traffic sign detection.
- **Wire.h**: Interfaces with the IMU sensor to gather orientation and acceleration data.
  
### **2. Code Workflow**
The software is designed to run an **event-driven** system where each sensor triggers a specific response. Below is the basic workflow:
1. **Initialization**: All sensors and motors are initialized. The vehicle waits for the start signal to begin the challenge.
2. **Obstacle Detection**: During navigation, the ultrasonic sensor continuously checks for obstacles. If an obstacle is detected within a certain range (e.g., 20 cm), the car halts and steers around the object before resuming its path.
3. **Lane Following**: The IR sensors detect the track’s boundaries. If the vehicle starts drifting outside the lane, corrective actions are taken by adjusting the steering angle via the servo motor.
4. **Traffic Sign Detection**: The TCS3200 color sensor scans for green and red traffic signs. A green sign instructs the vehicle to continue forward, while a red sign signals the vehicle to turn around and complete the final lap in the opposite direction.
5. **Parallel Parking**: Once all laps are complete, the vehicle uses the IMU and ultrasonic sensors to identify the parking lot and execute the parking maneuver by controlling both the steering and driving motors.

---

## **Compilation**

### **Steps to Build and Upload Code to Arduino:**
1. **Connect all hardware components** to the Arduino as per the circuit diagram (including motors, motor drivers, and sensors).
2. **Install the necessary libraries** in the Arduino IDE:
   - Go to **Sketch > Include Library > Manage Libraries** and install `Servo`, `NewPing`, `TCS3200`, and `Wire`.
3. **Load the code** onto the Arduino:
   - Open the Arduino IDE.
   - Select the correct board (`Arduino Uno`) under **Tools > Board**.
   - Select the correct port under **Tools > Port**.
   - Click the **Upload** button to upload the code to the Arduino.
4. **Test the sensors and motors**:
   - Use the **Serial Monitor** to debug sensor data and ensure the sensors are working correctly.
   - Test motor functionality to verify forward, backward, and turning movements.
5. **Run the full program** to start the vehicle in autonomous mode.

---

## **Challenges and Solutions**

### **1. Sensor Interference**
- **Challenge**: During initial testing, the ultrasonic sensor showed inconsistent readings when detecting obstacles, especially when the surface of the object was uneven or reflective.
- **Solution**: We adjusted the positioning of the sensor and fine-tuned the threshold values in the code to account for noisy data. Additionally, we added a delay between sensor readings to avoid rapid triggering.

### **2. Steering Calibration**
- **Challenge**: The servo motor responsible for steering had difficulty maintaining precise turns, particularly on sharp curves. It also resulted in some over-steering during the first trials.
- **Solution**: We implemented a proportional control algorithm that adjusts the steering angle based on the vehicle’s current position on the track. This allowed smoother turns and more controlled driving.

### **3. Battery Management**
- **Challenge**: The vehicle experienced power fluctuations when the motors and sensors drew high amounts of current simultaneously, which led to inconsistent behavior.
- **Solution**: We introduced a capacitor into the circuit to stabilize the power supply and used a voltage regulator to ensure consistent power delivery to the motors and sensors.

### **4. Traffic Sign Detection**
- **Challenge**: The color sensor initially had trouble differentiating between red and green traffic signs in varied lighting conditions.
- **Solution**: We programmed dynamic thresholding based on ambient light conditions. The vehicle now performs a calibration routine at the start of the run to adjust sensor sensitivity according to the lighting environment.
