# ESP32-Based IoT System with LED Control, Accelerometer, and Thermistor Monitoring


This project showcases a complete IoT solution using the ESP32 microcontroller. It integrates several peripherals such as an accelerometer (ADXL343), thermistor, and LED control via PWM, and communicates sensor data through a UDP server over WiFi.


# Key Features:
WiFi Communication: The ESP32 connects to a WiFi network and sets up a UDP server, enabling remote control and monitoring.
LED Control: Adjust the brightness of an LED using a PWM signal based on commands received via UDP.
Accelerometer (ADXL343) Integration: Reads real-time acceleration data from the ADXL343 sensor, calculates roll and pitch, and sends it to the UDP server.
Thermistor Temperature Sensing: Monitors and calculates temperature using a thermistor and displays the data in Celsius.
UDP Server: The system can receive commands for LED brightness and send sensor data (temperature and accelerometer readings) back to a remote client.
FreeRTOS Tasks: The project uses FreeRTOS tasks for multitasking, handling LED control, sensor monitoring, and WiFi communication in parallel.



# How It Works:
WiFi Setup: The ESP32 connects to a specified WiFi network and starts a UDP server.
Sensor Data Acquisition: Real-time data from the accelerometer and thermistor is collected and logged.
LED Control: The UDP server accepts commands to change the LED brightness or activate a cycling intensity mode.
Data Transmission: The sensor data is packaged and sent to the remote client after receiving a command.
I2C Device Scanning: Scans for connected I2C devices (such as the ADXL343) and initializes communication.



# Hardware:
ESP32 Development Board
ADXL343 Accelerometer
Thermistor
LED with PWM Control



#Future Improvements:
Add more sensors or actuators to extend the functionality.
Improve the user interface for remote control.
Implement advanced data processing and logging on the server side.
