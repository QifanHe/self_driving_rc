import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

Motor1A = 11
Motor1B = 12
Motor1E = 16

Motor2A = 13
Motor2B = 15
Motor2E = 18

GPIO.setup(Motor1A, GPIO.OUT)
GPIO.setup(Motor1B, GPIO.OUT)
GPIO.setup(Motor1E, GPIO.OUT)
GPIO.setup(Motor2A, GPIO.OUT)
GPIO.setup(Motor2B, GPIO.OUT)
GPIO.setup(Motor2E, GPIO.OUT)

motorL = GPIO.PWM(Motor1E,100)
motorR = GPIO.PWM(Motor2E,100)

motorL.start(0)
motorR.start(0)

GPIO.output(Motor1B, GPIO.LOW)
GPIO.output(Motor1A, GPIO.HIGH)
GPIO.output(Motor2A, GPIO.LOW)
GPIO.output(Motor2B, GPIO.HIGH)
motorR.ChangeDutyCycle(30)
motorL.ChangeDutyCycle(30)

time.sleep(3)
GPIO.output(Motor1B, GPIO.HIGH)
GPIO.output(Motor1A, GPIO.LOW)
time.sleep(3)
motorR.stop()
motorL.stop()
GPIO.cleanup()
