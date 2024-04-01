# PID in Brushed DC Motor using Rotary Encoder

In this project, I implement a PID (Proportional-Integral-Derivative) controller to control the position of the shaft in a Brushed DC motor using a rotary encoder. The goal is to give input for the desired position of the shaft, and then the motor shaft will rotate until it reaches that position.

It's important to note that I am not using an Encoder Motor; instead, I am utilizing a rotary encoder for this task. I am using a 300RPM (12V) DC motor, and if the speed exceeds 170 (speed in a scale of 0 to 255), this inexpensive encoder starts to exhibit issues. Therefore, I am limiting the maximum speed to 150 (speed in a scale of 0 to 255).

## Components Used:
1. Arduino Uno
2. Rotary Encoder
3. BTS Motor Driver (Other motor drivers can be used, but adjustments to the code may be necessary)
4. A DC Motor (I recommend a motor with less than 400RPM; higher RPM motors may cause the mechanical rotary encoder to fail to keep up with the speed, resulting in errors in the system)
5. Breadboard
6. Male to Female, Male to Male jumper wires
7. PVC or other material for housing all these components
8. 12V Power Supply Module

## Circuit Diagram:
![Circuit Diagram](https://github.com/tanvir-a0/PID_and_DC_Motor/assets/66798561/f0124bef-c668-448c-860d-64b4a97b928c)

## Final Setup Image:
![Final Setup](https://github.com/tanvir-a0/PID_and_DC_Motor/assets/66798561/1ad476de-5441-42eb-ba70-ee47930350e5)
