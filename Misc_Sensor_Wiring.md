## Wiring Added and Not Covered in the Youtube Tutorial ##

#### Battery Voltage ####
You can not send anything over 5v to the Analog pins on the arduino.  
To combat this, we need to wire the voltage to A6 on arduino as follows:

Car Battery + (9-17v)
     |
     |
    22kΩ
     |
     |
     +----------> A6  (Arduino Mega)
     |
     |
    10kΩ
     |
     |
    Ground
