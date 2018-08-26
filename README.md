# MPU6050
A simple library to setup the MPU6050 6 axis Gyro/Accelerometer

## Yet another MPU6050 project?
There are plenty of tutorials on how to setup the MPU6050. This project isn't intended to revolutionise this, it's for educational use.
Bellow I linked the tutorials I used to write it which you could use to understand how it is done.  

## Installation
Nothing too complicated for this. Include the MPU6050.h into your project and declare an MPU6050 object. In the set-up, call ConnectMPU folowed by CalibrateMPU and your MPU6050 should be ready.
In the loop function you have to call the UpdateMPU method and give it the amount of time that passed since the last time you called it. I'm using 
a timer interrupt but you can use what ever method you want.

The values are stored in the gyro_RPY member of the MPU6050.h.

### Hardware
- MPU6050
- Arduino that uses an ATmega328P chip.

#### Circuit
<img src="/Ressources/Images/Sketch.png" width="500">



## Credits
### Project contributors
Alexandre Doucet (_Doucet_)</br>


### External ressources
Most of my code comes from these tutorials. If you want check them out.

Joop Brokking <br/>
[![Link to youtube video](https://img.youtube.com/vi/4BoIE8YQwM8/0.jpg)](https://www.youtube.com/watch?v=4BoIE8YQwM8) <br/>
EEEnthusiast <br/>
[![Link to youtube video](https://img.youtube.com/vi/M9lZ5Qy5S2s/0.jpg)](https://www.youtube.com/watch?v=M9lZ5Qy5S2s)<br/>
