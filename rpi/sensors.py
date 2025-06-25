"""
Sensor module for Ultrasonic sensors.
Provides distance measurements.
"""

import RPi.GPIO as GPIO
import time

class UltrasonicSensor:
    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def get_distance(self):
        """
        Returns distance measurement in cm
        """
        # Send 10us pulse to trigger
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, False)

        start_time = time.time()
        stop_time = time.time()

        # Save start time
        while GPIO.input(self.echo_pin) == 0:
            start_time = time.time()

        # Save arrival time
        while GPIO.input(self.echo_pin) == 1:
            stop_time = time.time()

        # Time difference
        time_elapsed = stop_time - start_time
        # Calculate distance (speed of sound 34300 cm/s)
        distance = (time_elapsed * 34300) / 2

        return distance
