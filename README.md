# DoubloonDescrier_Robot
Arduino Code for the Doubloon Descrier Robot

# ====================================== ABOUT THE DOUBLOON DESCRIER ====================================== #
The Doubloon Descrier is a metal detecting robot that moves autonomously and detects metal through a metal
  detector array and deploy virtual/physical markers indicating where it found the object. The metal detector
  array sends a signal to the Arduino to deploy a physical marker from the 3D printed marker dropper connected
  to a stepper motor. This signal is also sent to the Doubloon Descrier iOS app via the Bluefruit Adafruit LE 
  SPI Friend and allows the app to drop a virtual marker on the in-app map by getting GPS coordinates from
  the connected NEO-6M GPS Module.
# ========================================================================================================= #

# ======================================== ABOUT THE ARDUINO CODE ========================================= #
This code allows you to program your own Doubloon Descrier Robot using an Arduino Mega.
It includes libraries for motor control (motor wheels and dropper), SPI, the Adafruit Bluefruit BLE library
  to allow you to communicate with your iOS device.
  
  *** DISCLAIMER ***
    This only includes the robot functionality and not the bluetooth functionality to connect to the
    Doubloon Descrier iOS app. This code connects to the BluefruitConnect app.
