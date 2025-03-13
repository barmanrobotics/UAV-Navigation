#Python script to control a stepper motor from limit switch


#importing GPIO library
#pip install gpiozero

from gpiozero import Button # type: ignore
from time import sleep
import RPi.GPIO as GPIO # type: ignore


#setting GPIO pins
limitSwitchPin = 23

#setting motor config
directionPin = 20
stepPin = 21
enablePin = 12 #set to None if not used
stepsPerRev = 200
leadscrewPitch = 8
targetDistance = 18 #in mm
stepsPerMM = stepsPerRev / leadscrewPitch
targetSteps = int(stepsPerMM * targetDistance)
delay = 0.002

#initializing stepper motor
GPIO.setmode(GPIO.BCM)
GPIO.setup(directionPin, GPIO.OUT)
GPIO.setup(stepPin, GPIO.OUT)
GPIO.setup(enablePin, GPIO.OUT)
GPIO.output(enablePin, GPIO.LOW)

#initializing limit switch
limitSwitch = Button(limitSwitchPin, pull_up=False)

#function to move stepper until limit switch is pressed
def moveStepper(steps, direction):
    GPIO.output(directionPin, direction)
    print("Moving stepper")
    for x in range(steps):
        if limitSwitch.is_pressed:
            print("Limit switch pressed, stopping stepper")
            break
        GPIO.output(stepPin, GPIO.HIGH)
        sleep(delay)
        GPIO.output(stepPin, GPIO.LOW)
        sleep(delay) 
    print("Stepper motor completed")

def extendStepperUntilLimit():
    moveStepper(targetSteps, GPIO.HIGH)

def retractStepperUntilLimit():
    moveStepper(targetSteps, GPIO.LOW)

