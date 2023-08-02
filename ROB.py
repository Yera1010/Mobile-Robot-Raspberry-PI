import time
import math
import smbus
import RPi.GPIO as GPIO
from gpiozero import RotaryEncoder
import numpy as np


# GPIO setmode
GPIO.setmode(GPIO.BCM)

# Define GPIO for Motors RIGHT SIDE
IN3 = 6
IN4 = 13
ENB = 26

# Define GPIO for Motors LEFT SIDE
IN1 = 4
IN2 = 27
ENA = 22

# Define RIGHT Encoder
ENCRA = 25
ENCRB = 5

# Define LEFT Encoder
ENCLA = 23
ENCLB = 24

# Set pins for Motors RIGHT SIDE
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Set pins for Motors LEFT SIDE
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Set pins for RIGHT Encoder
encoderRight = RotaryEncoder(ENCRA, ENCRB, max_steps=10)

# Set pins for LEFT Encoder
encoderLeft = RotaryEncoder(ENCLA, ENCLB, max_steps=10)

# Reset Driver pins
GPIO.output(IN1, GPIO.LOW)
GPIO.output(IN2, GPIO.LOW)
GPIO.output(IN3, GPIO.LOW)
GPIO.output(IN4, GPIO.LOW)
time.sleep(0.1)

# PWM
pA = GPIO.PWM(ENA, 100)
pB = GPIO.PWM(ENB, 100)
time.sleep(0.1)


# Set up directions for Robot motion

def forward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def backward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turnRight():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def turnLeft():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

wheelradius = 0.0325
wheeldistance = 0.18


ppr = 34
sampletime = 0.1
ToRight = 0
ToLeft = 0
wr = 0
wl = 0
vr = 0
vl = 0
xCurrent = 0
yCurrent = 0
distance = 5
theta = 0
V = 0
W = 0

stepsRightCurr = 0
stepsLeftCurr = 0
prevStepsRight = 0
prevStepsLeft = 0
tprev = 0.0
previousComputeTime = 0.0
errorX = 0.0
errorY = 0.0
errorTheta = 0
errorDistance = 0
# PID Right Motor
KP_R = 100.0
KI_R = 300.0
KD_R = 0.001
errorSpeedRightPrevious = 0.0
errorRSum = 0.0

# PID Left Motor
KP_L = 100.0
KI_L = 300.0
KD_L = 0.003
errorSpeedLeftPrevious = 0.0
errorLSum = 0.0

# Physical Constraints
maxLinearV = 0.3
maxAngularV = np.pi / 4

# Goals
XGoals = [1, 0, 1, 4, 0]  # Add more X coordinates as needed
YGoals = [0, 1, 0, 1, 1]   # Add more Y coordinates as needed

errorX = 0
errorY = 0
errorDistance = 0

Kdistance = 0.455
Ktheta = 0.62



try:

    
    pA.start(0)
    pB.start(0)
    stop()
    pA.ChangeDutyCycle(0)
    pB.ChangeDutyCycle(0)
    timeNow = time.perf_counter()
    tprev = timeNow
    previousComputeTime = timeNow
    goalIndex = 0  # Index of the current goal
    while goalIndex < len(XGoals):
        xGoal = XGoals[goalIndex]
        yGoal = YGoals[goalIndex]

        while True:
            # Read encoder steps
            stepsRight = encoderRight.steps
            stepsLeft = encoderLeft.steps

            # Reset encoder steps
            encoderRight.steps = 0
            encoderLeft.steps = 0

            # Calculate linear and angular velocity
            vr = (2 * np.pi * wheelradius * stepsRight) / 20
            vl = (2 * np.pi * wheelradius * stepsLeft) / 20
            V = (vr + vl) / 2.0
            W = (vr - vl) / wheeldistance

            # Calculate errors
            errorX = xGoal - xCurrent
            errorY = yGoal - yCurrent
            errorDistance = np.sqrt(errorX ** 2 + errorY ** 2)

            if errorDistance < 0.03:
                break

            # Calculate desired linear and angular velocity
            Vdesired = Kdistance * errorDistance
            Vdesired = np.clip(Vdesired, -maxLinearV, maxLinearV)

            thetaGoal = np.arctan2(errorY, errorX)
            thetaError = thetaGoal - theta
            thetaError = np.arctan2(np.sin(thetaError), np.cos(thetaError))

            Wdesired = Ktheta * thetaError
            Wdesired = np.clip(Wdesired, -maxAngularV, maxAngularV)

            # PID control for each wheel
            errorR = Vdesired + Wdesired * wheeldistance / 2.0 - vr
            errorL = Vdesired - Wdesired * wheeldistance / 2.0 - vl
            errorRSum += errorR
            errorLSum += errorL

            # PID control signal
            uR = KP_R * errorR + KI_R * errorRSum + KD_R * (errorR - errorSpeedRightPrevious)
            uL = KP_L * errorL + KI_L * errorLSum + KD_L * (errorL - errorSpeedLeftPrevious)

            # Motor power
            PWMR = np.absolute(uR)
            PWML = np.absolute(uL)

            PWMR = np.clip(PWMR, 0, 100)
            PWML = np.clip(PWML, 0, 100)

            # Set motor directions based on control signals
            if uR > 0 and uL > 0:
                forward()
            elif uR > 0 and uL < 0:
                turnRight()
            elif uR < 0 and uL > 0:
                turnLeft()
            elif uR < 0 and uL < 0:
                backward()

            # Update previous errors for next iteration
            errorSpeedRightPrevious = errorR
            errorSpeedLeftPrevious = errorL

            # Set motor power
            pA.ChangeDutyCycle(PWMR)
            pB.ChangeDutyCycle(PWML)

            # Update position using numerical integration (4th order Range-Kutta)
            dt = 0.0001  # Time step
            x2 = (
                    xCurrent
                    + dt
                    * V
                    * (2.0 + np.cos(dt * W / 2.0))
                    * np.cos(theta + dt * W / 2.0)
                    / 3.0
            )
            y2 = (
                    yCurrent
                    + dt
                    * V
                    * (2.0 + np.cos(dt * W / 2.0))
                    * np.sin(theta + dt * W / 2.0)
                    / 3.0
            )
            theta2 = theta + dt * W

            xCurrent = x2
            yCurrent = y2
            theta = theta2

            time.sleep(dt)

        # Stop the robot when reaching the goal
        stop()
        time.sleep(1)
        # Increment goal index
        goalIndex = goalIndex + 1 


except KeyboardInterrupt:
    stop()
    encoderRight.close()
    encoderLeft.close()
    GPIO.cleanup()

finally:
    stop()
    print("Done.")
    encoderRight.close()
    encoderLeft.close()
    GPIO.cleanup() 
    
    
