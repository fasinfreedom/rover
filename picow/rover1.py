import board, digitalio, pwmio
import time
import os
from hcsr04 import HCSR04

#Motor 1
#GP6 - ENA Speed Control
#GP7 - IN1 Direction Control
#GP8 - IN2 Direction Control

#Motor 2
#GP10 - ENA Speed Control
#GP11 - IN1 Direction Control
#GP12 - IN2 Direction Control

maxSpeed = 2 ** 16 - 1

class Motor:
    name = None
    in1 = None
    in2 = None
    en = None

    def __init__(self, name, in1, in2, en):
        self.name = name
        self.en = pwmio.PWMOut(en, duty_cycle = maxSpeed, frequency = 1500)
        self.in1 = digitalio.DigitalInOut(in1)
        self.in1.direction = digitalio.Direction.OUTPUT
        self.in2 = digitalio.DigitalInOut(in2)
        self.in2.direction = digitalio.Direction.OUTPUT

    # IN1 0 IN2 0 = Off
    # IN1 1 IN2 0 = Forward
    # IN1 0 IN2 1 = Backward

    def forward(self):
        self.in1.value = True
        self.in2.value = False

    def off(self):
        self.in1.value = False
        self.in2.value = False

    def speed(self, speed):
        self.en.duty_cycle = speed

    def on(self):
        self.forward()
        self.speed(maxSpeed)

    def print(self):
        print(f"\nMotor: {self.name}")
        print(f"En: {self.en.duty_cycle/65535*100}%")
        print(f"In1: {self.in1.value}")
        print(f"In2: {self.in2.value}")

green_led = digitalio.DigitalInOut(board.GP14)
green_led.direction = digitalio.Direction.OUTPUT

button = digitalio.DigitalInOut(board.GP15)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.DOWN

motors = [];
motor1 = Motor("Motor1", board.GP7, board.GP8, board.GP6)
motors.append(motor1)
motor2 = Motor("Motor2", board.GP11, board.GP12, board.GP10)
motors.append(motor2)

systemsOn = False

print("\nCOMMENCING PROGRAM")

ultrasonicTrig = board.GP2
ultrasonicEcho = board.GP3
systemStatusCurrent = False
sonar = HCSR04(ultrasonicTrig, ultrasonicEcho)

while True:
    if systemsOn:
        try:
            ultrasonicDistance = sonar.dist_cm()
            print(f"Distance: {ultrasonicDistance}")
            if ultrasonicDistance < 10:
                print("Close Object Detected")
        except KeyboardInterrupt:
            pass

    if systemStatusCurrent != button.value:
        print(f"\nSystem powering {'On' if button.value else 'Off'}")
        systemsOn = button.value
        systemStatusCurrent = systemsOn
        green_led.value = systemsOn
        for motor in motors:
            motor.on() if systemsOn else motor.off()
        print(f"Systems {'On' if systemsOn else 'Off'}")

        for motor in motors:
            motor.print()

    time.sleep(.1)
