from controller import Robot, Motor
from math import pi, sin

TIME_STEP = 32

robot = Robot()
motor1 = robot.getDevice("left wheel motor")
motor2 = robot.getDevice("right wheel motor")

F = 1.0   # frequency 2 Hz
t = 0.0   # elapsed simulation time

while robot.step(TIME_STEP) != -1:
    position = sin(t * 2.0 * pi * F)
    motor1.setPosition(-10)
    motor2.setPosition(10)
    t += TIME_STEP / 1000.0