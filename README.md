# DRIFT-ASSIST-SYSTEM
gyro based stability assist system. Uses an MPU6050(use my copied version of mpu6050 library) and an arduino mini pro and some jumper wires.

1)Preferred microcontroller: arduino mini pro 5V 16MHz
2)Preferred IMU : mpu6050
3)Orientation of mpu6050: Z axis facing vertically upwards

SETUP: 

1)connect steering channel from receiver to pin 8

2)connect throttle channel from receiver to pin 9

3)connect steering servo to pin 3

4)connect throttle servo/esc to pin 4

5)SDA of mpu to A4, SCL of mpu to A5 

Upon startup, give it a few seconds before giving in any inputs. It is using those few seconds to caliberate the gyro. 
The controller takes partial control of the throttle and the steering to prevent the car from oversteering too much. The amount of "help"
being given by the controller can be changed by reducing the Kd value and by multiplying "mod(G[2])" by some number smaller than 1.
