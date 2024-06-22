import board, digitalio, pwmio
import time
import os
from hcsr04 import HCSR04
import neopixel

#Motor 1
#GP11 - PWMA Speed Control
#GP14 - AIN1 Direction Control
#GP15 - AIN2 Direction Control

#Motor 2
#GP10 - PWMB Speed Control
#GP13 - BIN1 Direction Control
#GP12 - BIN2 Direction Control

maxSpeed = 2 ** 16 - 1

class Motor:
    name = None
    in1 = None
    in2 = None
    pwm = None

    def __init__(self, name, in1, in2, pwm):
        self.name = name
        self.pwm = pwmio.PWMOut(pwm, duty_cycle = maxSpeed, frequency = 1500)
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

    def reverse(self):
        self.in1.value = False
        self.in2.value = True

    def off(self):
        self.in1.value = False
        self.in2.value = False

    def speed(self, speed):
        self.pwm.duty_cycle = speed

    def on(self):
        self.forward()
        self.speed(maxSpeed)

    def print(self):
        print(f"\nMotor: {self.name}")
        print(f"PWM: {self.pwm.duty_cycle/65535*100}%")
        print(f"In1: {self.in1.value}")
        print(f"In2: {self.in2.value}")

def pixelOn(pixel):
    pixel.brightness = 0.2

def pixelOff(pixel):
    pixel.brightness = 0

pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
pixel.fill((50, 0, 255))
pixelOff(pixel)

button = digitalio.DigitalInOut(board.D2)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.DOWN

# motorStandby being False causes motors to be off
# motorStandby = digitalio.DigitalInOut(board.GP9)
# motorStandby.direction = digitalio.Direction.OUTPUT

motors = []
motor1 = Motor("Motor1", board.D9, board.D10, board.A5)
motors.append(motor1)
motor2 = Motor("Motor2", board.D11, board.D12, board.A4)
motors.append(motor2)

systemsOn = False
# motorStandby = True

print("\nCOMMENCING PROGRAM")

ultrasonicTrig = board.A1
ultrasonicEcho = board.A0

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
        pixelOn(pixel) if systemsOn else pixelOff(pixel)
        for motor in motors:
            motor.on() if systemsOn else motor.off()
        print(f"Systems {'On' if systemsOn else 'Off'}")

        for motor in motors:
            motor.print()

    time.sleep(.1)


def turnLeft(motors):
    motors[0].off()
    motors[1].on()

def turnRight(motors):
    motors[1].off()
    motors[0].on()

def newRoute(motors):
    turnLeft(motors)
