import board, digitalio, pwmio
from analogio import AnalogIn
import time
import os
from hcsr04 import HCSR04
import neopixel

maxSpeed = 0xFFFF

# Classes
# ========================================================================
class Motor:
    name = None
    in1 = None
    in2 = None
    pwm = None
    speed = maxSpeed

    def __init__(self, name, in1, in2, pwm):
        self.name = name
        self.pwm = pwmio.PWMOut(pwm, duty_cycle=self.speed, frequency=1000)
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

    def setSpeed(self, speed):
        self.speed = speed if speed >= 25 else 25
        self.pwm.duty_cycle = round(maxSpeed * self.speed / 100)

    def on(self):
        self.forward()

    def print(self):
        print(f"\nMotor: {self.name}")
        print(f"Speed: {self.speed}%")
        print(f"In1: {self.in1.value}")
        print(f"In2: {self.in2.value}")


# Functions
# ========================================================================
def pixelOn(pixel):
    pixel.brightness = 0.1

def pixelOff(pixel):
    pixel.brightness = 0

def pixelColor(pixel, color):
    if color == 'ORANGE':
        pixel.fill((255, 50, 0))
        return
    if color == 'RED':
        pixel.fill((255, 0, 0))
        return

def goStraight(motors):
    for motor in motors:
        motor.on()

def turnLeft(motors):
    motors[0].off()
    motors[1].on()

def turnRight(motors):
    motors[1].off()
    motors[0].on()

def newRoute(motors):
    turnLeft(motors)

def speedCheck(speedControlCurrent, speedControl, motors):
    if abs(speedControlCurrent - speedControl.value) >= 0xFFFF * .005:
        speedControlCurrent = speedControl.value
        motorsSpeed = round(speedControlCurrent / 0xFFFF * 100)
        for motor in motors:
            motor.setSpeed(motorsSpeed)
        print(f"Speed Control: {motorsSpeed}%")

    return speedControlCurrent


# Main Program
# ========================================================================
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
pixel.fill((255, 50, 0))
pixelOff(pixel)

button = digitalio.DigitalInOut(board.D2)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.DOWN

# motorsEnable set to false causes motor controller to go into Standby
motorsEnable = digitalio.DigitalInOut(board.D11)
motorsEnable.direction = digitalio.Direction.OUTPUT
motorsEnable = True
motorsSpeed = 50 #25-100
motorsDirection = 'STRAIGHT'

motors = []
motor1 = Motor("Motor1", board.D5, board.D6, board.A5)
motor1.setSpeed(motorsSpeed)
motors.append(motor1)
motor2 = Motor("Motor2", board.D9, board.D10, board.A4)
motor2.setSpeed(motorsSpeed)
motors.append(motor2)

systemsOn = False
print("\nCOMMENCING PROGRAM")

sonarTrig = board.A1
sonarEcho = board.A0

systemStatusCurrent = False
sonar = HCSR04(sonarTrig, sonarEcho)
sonarDistance = 0

speedControl = AnalogIn(board.A2)
speedControlCurrent = 0

while True:
    if systemsOn:
        try:
            sonarDistanceNew = round(sonar.dist_cm())
            if sonarDistance != sonarDistanceNew:
                print(f"Distance: {sonarDistance} cm")
                pixelColor(pixel, 'ORANGE')
                if sonarDistance < 10:
                    print("Close Object Detected")
                    pixelColor(pixel, 'RED')
                    if motorsDirection != 'LEFT':
                        turnLeft(motors)
                        motorsDirection = 'LEFT'
                        print(f"Direction: {motorsDirection}")
                else:
                    if motorsDirection != 'STRAIGHT':
                        goStraight(motors)
                        motorsDirection = 'STRAIGHT'
                        print(f"Direction: {motorsDirection}")
                sonarDistance = sonarDistanceNew
            speedControlCurrent = speedCheck(speedControlCurrent, speedControl, motors)

        except KeyboardInterrupt:
            pass

    if systemStatusCurrent != button.value:
        systemsOn = button.value
        print(f"\nSystem powering {'On' if button.value else 'Off'}")
        systemStatusCurrent = systemsOn
        pixelOn(pixel) if systemsOn else pixelOff(pixel)
        for motor in motors:
            motor.on() if systemsOn else motor.off()
        print(f"Systems {'On' if systemsOn else 'Off'}")

        print(f"Motors {'Enabled' if motorsEnable else 'Disable'}")
        for motor in motors:
            motor.print()

    time.sleep(0.1)
