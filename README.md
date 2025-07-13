# smart_parking_system
Smart parking system using Nuvoton NUC140 microcontroller with RFID, ultrasonic sensors, servo control, and Bluetooth mobile app integration.




## 🔧 How It Works

- At the **entrance**, a **distance sensor (HC-SR04)** detects the presence of a vehicle.
- Once detected, the system checks the following conditions **before allowing entry**:
  1. The vehicle is at a distance of less than 70 mm from the sensor (i.e., close to the gate).
  2. A valid **RFID tag** assigned to the vehicle is scanned.
  3. The current **car count is less than 3** (i.e., parking space is available).
- If all conditions are satisfied, the **servo motor** activates to open the gate and allows the car to enter.
- At the **exit**, another **ultrasonic sensor** detects a vehicle waiting to leave.
- The gate will only open if a **payment confirmation** is received via the **Android app**.





## 🎥 Demo Video

Watch a live demonstration of the Smart Parking System in action:  
👉 [Smart Parking System Demo on YouTube](https://youtu.be/IOieSrHeTKE)




## 📱 Android App

The system includes a custom Android application built with **MIT App Inventor**.  
The app allows the user to:
- Confirm and send payment for exiting the parking lot.
- View parking status .
- Communicate with the system via **Bluetooth (HC-05)**.
📦 App source file  is included.



## 🧰 Tools Used

- Keil µVision (C programming)
- MIT App Inventor (Android app)
- Nuvoton NUC140 development board


## 🧠 Main Features

- RFID-based vehicle authentication
- Entry and exit managed with ultrasonic sensors
- Servo-controlled gate mechanism
- LCD feedback for system status
- Bluetooth communication with Android app
- Exit authorization via in-app payment


## 👨‍🔧 Author

- Naor David  
