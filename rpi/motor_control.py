"""
Motor control module for 4 propellers using L298N motor drivers.
Controls 4 motors independently: left/right for movement and front/back for vertical control.
Each L298N driver controls 2 motors using all 4 inputs (IN1-IN4) and both enables (ENA, ENB).
"""

try:
    import RPi.GPIO as GPIO
except ImportError:
    raise ImportError("RPi.GPIO module not found. Please install it for Raspberry Pi hardware.")
import time

class MotorControl:
    def __init__(self):
        # Clean up any existing GPIO configuration
        try:
            GPIO.cleanup()
        except Exception as e:
            print(f"Warning: GPIO cleanup failed: {str(e)}")
        
        # Initialize GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        
        # Stop any existing PWM
        try:
            self.cleanup_pwm()
        except Exception as e:
            print(f"Warning: PWM cleanup failed: {str(e)}")
            
        # Set motors to continuously run DOWN to submerge
        try:
            # Front Motor - GPIO 22 (UP) OFF, GPIO 23 (DOWN) ON
            GPIO.setup(22, GPIO.OUT)
            GPIO.output(22, GPIO.LOW)  # Turn OFF UP
            GPIO.setup(23, GPIO.OUT)
            GPIO.output(23, GPIO.HIGH)  # Turn ON DOWN
            
            # Back Motor - GPIO 12 (UP) OFF, GPIO 13 (DOWN) ON
            GPIO.setup(12, GPIO.OUT)
            GPIO.output(12, GPIO.LOW)  # Turn OFF UP
            GPIO.setup(13, GPIO.OUT)
            GPIO.output(13, GPIO.HIGH)  # Turn ON DOWN
            
            # Start PWM at full speed for submerging
            self.init_pwm()
            self.front_pwm.ChangeDutyCycle(100)
            self.back_pwm.ChangeDutyCycle(100)
            
            print("L298N #2 motors set to continuously run DOWN for submerging")
        except Exception as e:
            print(f"Critical: Failed to set motors to continuous DOWN: {str(e)}")
        
        # L298N #1 - Forward/Backward Motors
        # Motor A (Left)
        self.LEFT_IN1 = 17
        self.LEFT_IN2 = 18
        self.LEFT_ENA = 27
        # Motor B (Right)
        self.RIGHT_IN3 = 16
        self.RIGHT_IN4 = 19
        self.RIGHT_ENB = 26
        
        # L298N #2 - Up/Down Motors
        # Motor A (Front)
        self.FRONT_IN1 = 22  # Up movement
        self.FRONT_IN2 = 23  # Down movement
        self.FRONT_ENA = 24  # Speed control
        # Motor B (Back)
        self.BACK_IN3 = 12
        self.BACK_IN4 = 13
        self.BACK_ENB = 6

        # Setup all GPIO pins with connection verification
        print("\nInitializing GPIO pins...")
        
        # Test L298N #2 motor connections
        print("\nTesting L298N #2 Motors connections:")
        
        # Front Motor
        front_motor_pins = {
            'FRONT_IN1 (Up)': self.FRONT_IN1,
            'FRONT_IN2 (Down)': self.FRONT_IN2,
            'FRONT_ENA (Speed)': self.FRONT_ENA
        }
        
        # Back Motor
        back_motor_pins = {
            'BACK_IN3 (Up)': self.BACK_IN3,
            'BACK_IN4 (Down)': self.BACK_IN4,
            'BACK_ENB (Speed)': self.BACK_ENB
        }
        
        # Initialize Front Motor
        print("\nFront Motor:")
        for name, pin in front_motor_pins.items():
            try:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
                print(f"✓ {name} (GPIO {pin}) initialized successfully")
            except Exception as e:
                print(f"✗ {name} (GPIO {pin}) initialization failed: {str(e)}")
                if pin == self.FRONT_IN2:
                    print("  Note: If FRONT_IN2 (GPIO 23) is intentionally disconnected, this warning can be ignored")
        
        # Initialize Back Motor
        print("\nBack Motor:")
        for name, pin in back_motor_pins.items():
            try:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
                print(f"✓ {name} (GPIO {pin}) initialized successfully")
                
                # Ensure ENB (Speed Control) is active
                if pin == self.BACK_ENB:
                    GPIO.output(pin, GPIO.HIGH)
                    print(f"✓ {name} (GPIO {pin}) activated for continuous operation")
            except Exception as e:
                print(f"✗ {name} (GPIO {pin}) initialization failed: {str(e)}")
        
        # Setup remaining pins
        other_pins = [
            self.LEFT_IN1, self.LEFT_IN2, self.LEFT_ENA,
            self.RIGHT_IN3, self.RIGHT_IN4, self.RIGHT_ENB,
            self.BACK_IN3, self.BACK_IN4, self.BACK_ENB
        ]
        
        print("\nInitializing other motor pins:")
        for pin in other_pins:
            try:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
            except Exception as e:
                print(f"Warning: Failed to initialize GPIO {pin}: {str(e)}")
                
        # Double check critical pins
        try:
            # Check Front Motor (GPIO 22)
            if GPIO.input(22) == GPIO.HIGH:
                print("Warning: GPIO 22 still HIGH, forcing LOW again")
                GPIO.output(22, GPIO.LOW)
            
            # Check Back Motor (GPIO 12)
            if GPIO.input(12) == GPIO.HIGH:
                print("Warning: GPIO 12 still HIGH, forcing LOW again")
                GPIO.output(12, GPIO.LOW)
                
            # Ensure Back Motor ENB is HIGH
            if GPIO.input(self.BACK_ENB) == GPIO.LOW:
                print("Warning: Back Motor ENB LOW, activating again")
                GPIO.output(self.BACK_ENB, GPIO.HIGH)
        except Exception as e:
            print(f"Error checking motor states: {str(e)}")
        
        print("\nGPIO initialization completed")

        # Initialize PWM for all motors
        self.init_pwm()

    def forward(self, speed: int):
        """Move forward at given speed (0-100) - both left and right motors"""
        self._set_motor(self.LEFT_IN1, self.LEFT_IN2, self.left_pwm, True, speed)
        self._set_motor(self.RIGHT_IN3, self.RIGHT_IN4, self.right_pwm, True, speed)

    def backward(self, speed: int):
        """Move backward at given speed (0-100) - both left and right motors"""
        self._set_motor(self.LEFT_IN1, self.LEFT_IN2, self.left_pwm, False, speed)
        self._set_motor(self.RIGHT_IN3, self.RIGHT_IN4, self.right_pwm, False, speed)

    def turn_left(self, speed: int):
        """Turn left - right motor forward, left motor backward"""
        self._set_motor(self.LEFT_IN1, self.LEFT_IN2, self.left_pwm, False, speed)
        self._set_motor(self.RIGHT_IN3, self.RIGHT_IN4, self.right_pwm, True, speed)

    def turn_right(self, speed: int):
        """Turn right - left motor forward, right motor backward"""
        self._set_motor(self.LEFT_IN1, self.LEFT_IN2, self.left_pwm, True, speed)
        self._set_motor(self.RIGHT_IN3, self.RIGHT_IN4, self.right_pwm, False, speed)

    def up(self, speed: int):
        """Return to float state by activating UP pins"""
        print(f"MotorControl.up called with speed {speed}")
        
        # Front Motor - Activate UP pin (IN1)
        print(f"Front Motor: Activating UP pin GPIO 22")
        GPIO.output(self.FRONT_IN2, GPIO.LOW)   # Turn off DOWN
        GPIO.output(self.FRONT_IN1, GPIO.HIGH)  # Turn on UP
        self.front_pwm.ChangeDutyCycle(speed)
        
        # Back Motor - Activate UP pin (IN3)
        print(f"Back Motor: Activating UP pin GPIO 12")
        GPIO.output(self.BACK_IN4, GPIO.LOW)    # Turn off DOWN
        GPIO.output(self.BACK_IN3, GPIO.HIGH)   # Turn on UP
        self.back_pwm.ChangeDutyCycle(speed)

    def down(self, speed: int):
        """Activate DOWN pins for continuous submerge"""
        print(f"MotorControl.down called with speed {speed}")
        
        try:
            # Front Motor - Activate DOWN pin (IN2)
            print(f"Front Motor: Activating DOWN pin GPIO 23")
            GPIO.output(self.FRONT_IN1, GPIO.LOW)   # Turn off UP
            GPIO.output(self.FRONT_IN2, GPIO.HIGH)  # Turn on DOWN
            self.front_pwm.ChangeDutyCycle(speed)
            
            # Back Motor - Activate DOWN pin (IN4)
            print(f"Back Motor: Activating DOWN pin GPIO 13")
            GPIO.output(self.BACK_IN3, GPIO.LOW)    # Turn off UP
            GPIO.output(self.BACK_IN4, GPIO.HIGH)   # Turn on DOWN
            self.back_pwm.ChangeDutyCycle(speed)
            
            print(f"Vertical motors running at {speed}% speed")
        except Exception as e:
            print(f"Error activating vertical motors: {e}")
            # Try to recover
            try:
                GPIO.output(self.FRONT_IN2, GPIO.HIGH)
                GPIO.output(self.BACK_IN4, GPIO.HIGH)
                self.front_pwm.ChangeDutyCycle(speed)
                self.back_pwm.ChangeDutyCycle(speed)
            except:
                print("Failed to recover vertical motors")

    def pitch_up(self, speed: int):
        """Pitch up - front motor down, back motor up"""
        self._set_motor(self.FRONT_IN1, self.FRONT_IN2, self.front_pwm, False, speed)
        self._set_motor(self.BACK_IN3, self.BACK_IN4, self.back_pwm, True, speed)

    def pitch_down(self, speed: int):
        """Pitch down - front motor up, back motor down"""
        self._set_motor(self.FRONT_IN1, self.FRONT_IN2, self.front_pwm, True, speed)
        self._set_motor(self.BACK_IN3, self.BACK_IN4, self.back_pwm, False, speed)

    def _set_motor(self, in1_pin: int, in2_pin: int, pwm_obj, forward: bool, speed: int):
        """Helper method to set individual motor direction and speed with enhanced error handling"""
        try:
            print(f"_set_motor called with in1_pin={in1_pin}, in2_pin={in2_pin}, forward={forward}, speed={speed}")
            
            # Validate speed range
            if not 0 <= speed <= 100:
                print(f"Warning: Speed value {speed} out of range (0-100), clipping to valid range")
                speed = max(0, min(speed, 100))
            
            # Set motor direction with error checking
            try:
                if forward:
                    # For up movement
                    GPIO.output(in1_pin, GPIO.HIGH)
                    # Check if IN2 pin is properly connected (for front motor)
                    if in2_pin == 23:  # Front motor IN2
                        try:
                            GPIO.output(in2_pin, GPIO.LOW)
                        except:
                            print(f"Warning: Unable to set IN2 (GPIO {in2_pin}) to LOW. Pin might be disconnected.")
                            # Force IN1 LOW to prevent reverse operation
                            GPIO.output(in1_pin, GPIO.LOW)
                            return
                    else:
                        GPIO.output(in2_pin, GPIO.LOW)
                else:
                    # For down movement
                    GPIO.output(in1_pin, GPIO.LOW)
                    if in2_pin == 23:  # Front motor IN2
                        try:
                            GPIO.output(in2_pin, GPIO.HIGH)
                        except:
                            print(f"Warning: Unable to set IN2 (GPIO {in2_pin}) to HIGH. Pin might be disconnected.")
                            return
                    else:
                        GPIO.output(in2_pin, GPIO.HIGH)
                
                # Set PWM speed
                pwm_obj.ChangeDutyCycle(speed)
                print(f"GPIO pins set: {in1_pin}={'HIGH' if forward else 'LOW'}, {in2_pin}={'LOW' if forward else 'HIGH'}, PWM duty cycle set to {speed}")
                
            except Exception as e:
                print(f"Error setting motor direction: {str(e)}")
                # Safety: Stop motor on error
                self.force_stop_motor(in1_pin, in2_pin, pwm_obj)
                
        except Exception as e:
            print(f"Critical error in _set_motor: {str(e)}")
            # Attempt emergency stop
            try:
                self.force_stop_motor(in1_pin, in2_pin, pwm_obj)
            except:
                print("Emergency stop failed!")

    def force_stop_motor(self, in1_pin, in2_pin, pwm_obj):
        """Force stop a single motor by setting everything to LOW with enhanced error handling"""
        try:
            # First set direction pins LOW with special handling for GPIO 23
            GPIO.output(in1_pin, GPIO.LOW)
            
            if in2_pin == 23:  # Front motor IN2
                try:
                    GPIO.output(in2_pin, GPIO.LOW)
                except:
                    print(f"Warning: Unable to set IN2 (GPIO {in2_pin}) to LOW. Pin might be disconnected.")
            else:
                GPIO.output(in2_pin, GPIO.LOW)
            
            # Handle PWM differently for Back Motor
            if in1_pin == self.BACK_IN3 and in2_pin == self.BACK_IN4:
                # For Back Motor, keep ENB HIGH but set speed to 0
                pwm_obj.ChangeDutyCycle(0)
                GPIO.output(self.BACK_ENB, GPIO.HIGH)
                print(f"Back Motor stopped but ENB kept HIGH for continuous operation")
            else:
                # For other motors, set PWM to 0
                pwm_obj.ChangeDutyCycle(0)
            
            # Ensure pins are still in OUTPUT mode
            GPIO.setup(in1_pin, GPIO.OUT)
            if in2_pin != 23:  # Skip GPIO 23 if it's disconnected
                GPIO.setup(in2_pin, GPIO.OUT)
                
            print(f"Motor stopped: IN1={in1_pin}, IN2={in2_pin}")
            
        except Exception as e:
            print(f"Error in force_stop_motor: {str(e)}")

    def stop(self):
        """Stop all motors completely"""
        print("MotorControl.stop called: stopping all motors")
        
        try:
            # Stop all motors
            print("Stopping forward/backward motors...")
            self.force_stop_motor(self.LEFT_IN1, self.LEFT_IN2, self.left_pwm)
            self.force_stop_motor(self.RIGHT_IN3, self.RIGHT_IN4, self.right_pwm)
            
            print("Stopping vertical motors...")
            # Front Motor
            GPIO.output(self.FRONT_IN1, GPIO.LOW)
            GPIO.output(self.FRONT_IN2, GPIO.LOW)
            self.front_pwm.ChangeDutyCycle(0)
            
            # Back Motor
            GPIO.output(self.BACK_IN3, GPIO.LOW)
            GPIO.output(self.BACK_IN4, GPIO.LOW)
            self.back_pwm.ChangeDutyCycle(0)
            
            print("All motors stopped completely")
            
        except Exception as e:
            print(f"Error in stop: {str(e)}")
            # Emergency handling - force everything to stop
            try:
                # Force all pins LOW
                for pin in [self.LEFT_IN1, self.LEFT_IN2, self.RIGHT_IN3, self.RIGHT_IN4,
                           self.FRONT_IN1, self.FRONT_IN2, self.BACK_IN3, self.BACK_IN4]:
                    GPIO.output(pin, GPIO.LOW)
                # Force all PWM to 0
                for pwm in [self.left_pwm, self.right_pwm, self.front_pwm, self.back_pwm]:
                    pwm.ChangeDutyCycle(0)
            except:
                print("Emergency stop failed!")

    def move_forward_down(self, speed: int):
        """Move forward and down simultaneously at given speed (0-100)"""
        # Forward motors forward
        self._set_motor(self.LEFT_IN1, self.LEFT_IN2, self.left_pwm, True, speed)
        self._set_motor(self.RIGHT_IN3, self.RIGHT_IN4, self.right_pwm, True, speed)
        # Down motors down (invert front motor direction)
        self._set_motor(self.FRONT_IN1, self.FRONT_IN2, self.front_pwm, False, speed)  # corrected direction
        self._set_motor(self.BACK_IN3, self.BACK_IN4, self.back_pwm, False, speed)

    def cleanup_pwm(self):
        """Stop and cleanup all PWM objects"""
        try:
            if hasattr(self, 'left_pwm'):
                self.left_pwm.stop()
            if hasattr(self, 'right_pwm'):
                self.right_pwm.stop()
            if hasattr(self, 'front_pwm'):
                self.front_pwm.stop()
            if hasattr(self, 'back_pwm'):
                self.back_pwm.stop()
        except:
            pass

    def init_pwm(self):
        """Initialize PWM for all motors"""
        # Setup PWM for speed control (1kHz frequency)
        self.left_pwm = GPIO.PWM(self.LEFT_ENA, 1000)
        self.right_pwm = GPIO.PWM(self.RIGHT_ENB, 1000)
        self.front_pwm = GPIO.PWM(self.FRONT_ENA, 1000)
        self.back_pwm = GPIO.PWM(self.BACK_ENB, 1000)

        # Start all PWM with 0% duty cycle
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        self.front_pwm.start(0)
        self.back_pwm.start(0)
        print("PWM started for all motors: left, right, front, back")

    def cleanup(self):
        """Cleanup GPIO and PWM with special handling for Back Motor"""
        print("Starting cleanup sequence...")
        
        try:
            # First ensure Back Motor ENB is HIGH
            GPIO.output(self.BACK_ENB, GPIO.HIGH)
            print("Back Motor ENB set HIGH before cleanup")
            
            # Stop all motors (this will keep Back Motor ENB HIGH)
            self.stop()
            
            # Cleanup PWM except for Back Motor
            try:
                self.left_pwm.stop()
                self.right_pwm.stop()
                self.front_pwm.stop()
                # Don't stop back_pwm to keep ENB active
                print("PWM cleanup completed (except Back Motor)")
            except Exception as e:
                print(f"Warning during PWM cleanup: {str(e)}")
            
            # Don't do full GPIO cleanup as it would reset Back Motor ENB
            # Instead, cleanup specific pins except Back Motor pins
            non_back_motor_pins = [
                self.LEFT_IN1, self.LEFT_IN2, self.LEFT_ENA,
                self.RIGHT_IN3, self.RIGHT_IN4, self.RIGHT_ENB,
                self.FRONT_IN1, self.FRONT_IN2, self.FRONT_ENA
            ]
            
            for pin in non_back_motor_pins:
                try:
                    GPIO.cleanup(pin)
                except:
                    pass
                    
            print("Cleanup completed while maintaining Back Motor ENB state")
            
        except Exception as e:
            print(f"Error during cleanup: {str(e)}")

    def move_down_both(self, speed: int):
        """Move both front and back motors down at given speed (0-100)"""
        print(f"move_down_both called with speed {speed}")
        print(f"Activating front motor pins: IN1={self.FRONT_IN1}, IN2={self.FRONT_IN2}, PWM={self.FRONT_ENA}")
        self._set_motor(self.FRONT_IN1, self.FRONT_IN2, self.front_pwm, False, speed)
        print(f"Activating back motor pins: IN3={self.BACK_IN3}, IN4={self.BACK_IN4}, PWM={self.BACK_ENB}")
        self._set_motor(self.BACK_IN3, self.BACK_IN4, self.back_pwm, False, speed)

    def move_all_forward(self, speed: int):
        """Move all four motors forward at given speed (0-100)"""
        print(f"move_all_forward called with speed {speed}")
        self._set_motor(self.LEFT_IN1, self.LEFT_IN2, self.left_pwm, True, speed)
        self._set_motor(self.RIGHT_IN3, self.RIGHT_IN4, self.right_pwm, True, speed)
        self._set_motor(self.FRONT_IN1, self.FRONT_IN2, self.front_pwm, True, speed)
        self._set_motor(self.BACK_IN3, self.BACK_IN4, self.back_pwm, True, speed)
