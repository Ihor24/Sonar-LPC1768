# Sonar-LPC1768

## Description
The project consists of the design of an embedded system based on the LPC1768 microcontroller. There is two operating mode: 

1. In a manual mode the position of the servo is controlled by the Key1 and Key2 pushbuttons so that  will cause the rotation of 10º to the left or right. The ISP button will start the distance measurement process over the set position. The distance measurement will update every 0.5s. If during this measurement process you press again the ISP it will stop the measurement. During the time when the system is measuring distances it doesn't respond to the pushbuttons that set the position of the servo.

2. In automatic sweep mode, sonar is sweeping continuously at a given initial resolution of 10º, with regular intervals of 0.5s. To temporarily stop this process, simply press ISP, restarting the scan after a new click. To enter the auto-scan mode it will be necessary to make a RESET by holding Key1.

In both modes, the system is able to detect obstacles at a configurable distance between 30cm and 150cm, generating an alarm signal depending on the intervals at which the obstacle is encountered. The serial interface will design an options menu that can be accessed by pressing "h" so that, within any mode, you can stop "off"/start "on" the scan, modify some control parameter, the accuracy of the servo motor and the period of movement and display the measurements obtained in ASCII format on a PC with the help of a terminal program.

## Components
- Microcontroller: LPC1768
- Distance Sensor: HC SR04
- Amplifier: PAM8403
- Microphone: MAX9814
- Servo Motor: SG90
