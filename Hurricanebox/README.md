#  Hurricane Box

Authors: Karl Carisme

Date: 2024-04-30

### Summary

The "Hurricane Box" project integrates an ESP32 microcontroller with sensors to monitor environmental conditions indicative of hurricane activity, including temperature and accelerations. This data is processed by the ESP32 and transmitted via WiFi to a node.js server, which facilitates real-time data communication and device control. Users can interact with the Hurricane Box through a web interface that displays sensor data and allows for remote adjustment of an LED's brightness as a means of testing system responsiveness. The web interface is built using HTML, CSS, and JavaScript, ensuring it is user-friendly and accessible from various devices.

 
### Solution Design
Thermistor, ADXL343, accelerometer, and LED was used.
Data is read through the ESP32 pins and controlled via udp. Data send to the web interface using nodejs. Data display on the webbrower on a html. Socketio was used. Port forwarding is done by no-ip DDNS service. karlchost.zapto.org:3000



### Supporting Artifacts
- [Link to video demo](https://drive.google.com/file/d/1CboeyoxzMKRxfByJnj9JM2KxhB83Ec0Q/view?usp=sharing).

### Modules, Tools, Source Used Including Attribution
ESP32
Thermistor
ADC
Accelelometer
i2c
UDP Protocol
DDNS
PWM
node.js (socket.io, express0

### References
https://www.w3schools.com/howto/howto_js_rangeslider.asp

https://www.w3schools.com/jsref/met_document_getelementbyid.asp
