import board, digitalio, pwmio
import time
import os


#Motor 1
#GP6 - ENA Speed Control
#GP7 - IN1 Direction Control
#GP8 - IN2 Direction Control

#Motor 2
#GP10 - ENA Speed Control
#GP11 - IN1 Direction Control
#GP12 - IN2 Direction Control

# IN1 0 IN2 0 = Off
# IN1 1 IN2 0 = Forward
# IN1 0 IN2 1 = Backward

maxSpeed = 2 ** 16 - 1

def motorForward(in1, in2):
    in1.value = True
    in2.value = False

def motorOff(in1, in2):
    in1.value = False
    in2.value = False

def motorSpeed(en, speed):
    en.duty_cycle = speed

def motor1Power(run):
    if run:
        print("Motor 1 On")
        motorForward(motor1In1, motor1In2)
        motorSpeed(motor1Ena, maxSpeed)
    else:
        print("Motor 1 Off")
        motorOff(motor1In1, motor1In2)

def motor2Power(run):
    if run:
        print("Motor 2 On")
        motorForward(motor2In1, motor2In2)
        motorSpeed(motor2Ena, maxSpeed)
    else:
        print("Motor 2 Off")
        motorOff(motor2In1, motor2In2)

def motorPrintInfo():
    print(f"Motor1 Ena: {motor1Ena}")
    print(f"Motor1 In1: {motor1In1.value}")
    print(f"Motor1 In2: {motor1In2.value}")

    print(f"Motor2 Ena: {motor2Ena}")
    print(f"Motor2 In1: {motor2In1.value}")
    print(f"Motor2 In2: {motor2In2.value}")

green_led = digitalio.DigitalInOut(board.GP14)
green_led.direction = digitalio.Direction.OUTPUT

button = digitalio.DigitalInOut(board.GP15)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.DOWN

motor1Ena = pwmio.PWMOut(board.GP6, duty_cycle = maxSpeed, frequency = 1500)

motor1In1 = digitalio.DigitalInOut(board.GP7)
motor1In1.direction = digitalio.Direction.OUTPUT

motor1In2 = digitalio.DigitalInOut(board.GP8)
motor1In2.direction = digitalio.Direction.OUTPUT

motor2Ena = pwmio.PWMOut(board.GP10, duty_cycle = maxSpeed, frequency = 1500)

motor2In1 = digitalio.DigitalInOut(board.GP11)
motor2In1.direction = digitalio.Direction.OUTPUT

motor2In2 = digitalio.DigitalInOut(board.GP12)
motor2In2.direction = digitalio.Direction.OUTPUT

systemsOn = False

print("\nCOMMENCING PROGRAM")
while True:
    if button.value:
        systemsOn = not systemsOn
        green_led.value = systemsOn
        motor1Power(systemsOn)
        motor2Power(systemsOn)
        print(f"\nSystems {'On' if systemsOn else 'Off'}")
        motorPrintInfo()
        time.sleep(1.5)
    time.sleep(.1)
