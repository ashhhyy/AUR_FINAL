"""
Autonomous logic module for underwater robot.
Implements box-type movement with obstacle avoidance.
"""

import time
import threading

class AutonomousLogic:
    def __init__(self, motor_control, front_sensor, back_sensor, bottom_sensor):
        self.motor_control = motor_control
        self.front_sensor = front_sensor
        self.back_sensor = back_sensor
        self.bottom_sensor = bottom_sensor
        self.obstacle_threshold = 5  # cm - stop and turn if obstacle within 5cm
        self.mission_time = 30  # seconds for box movement
        self.stop_event = threading.Event()
        self.current_direction = 0  # 0: forward, 1: right, 2: backward, 3: left

    def check_front_obstacle(self):
        """Check if there's an obstacle in front"""
        try:
            front_dist = self.front_sensor.get_distance()
            print(f"Front distance: {front_dist}cm")
            return front_dist < self.obstacle_threshold
        except Exception as e:
            print(f"Error reading front sensor: {e}")
            return False

    def avoid_obstacle(self):
        """Avoid obstacle by turning right"""
        print("Obstacle detected! Turning right to avoid...")
        self.motor_control.stop()
        time.sleep(0.5)
        self.motor_control.turn_right(70)
        time.sleep(1)
        self.motor_control.forward(70)

    def box_movement(self):
        """Execute one side of box movement"""
        movement_time = 2  # seconds per side
        print(f"Moving in direction {self.current_direction}")
        
        start_time = time.time()
        while time.time() - start_time < movement_time:
            if self.stop_event.is_set():
                return False
            
            # Check for obstacles only when moving forward
            if self.current_direction == 0:
                if self.check_front_obstacle():
                    self.avoid_obstacle()
                    return True
            
            # Execute movement based on current direction
            if self.current_direction == 0:
                print("Moving forward")
                self.motor_control.forward(70)
            elif self.current_direction == 1:
                print("Turning right")
                self.motor_control.turn_right(70)
            elif self.current_direction == 2:
                print("Moving backward")
                self.motor_control.backward(70)
            elif self.current_direction == 3:
                print("Turning left")
                self.motor_control.turn_left(70)
            
            # Keep vertical motors running at 70%
            self.motor_control.down(70)
            time.sleep(0.1)
        
        # Stop horizontal motors but keep vertical motors running
        self.motor_control.stop()
        time.sleep(0.5)
        
        # Update direction for next movement
        self.current_direction = (self.current_direction + 1) % 4
        print(f"Completed movement, next direction: {self.current_direction}")
        return True

    def run(self):
        """Non-blocking run method using state machine approach"""
        def mission():
            try:
                start_time = time.time()
                
                # Initial submerge and keep vertical motors running
                print("Initial submerge and activating continuous vertical motors...")
                self.motor_control.down(70)  # Initial submerge
                time.sleep(1)
                
                print("Starting box-type movement...")
                while not self.stop_event.is_set():
                    # Check if mission time (30 seconds) has elapsed
                    elapsed_time = time.time() - start_time
                    if elapsed_time >= self.mission_time:
                        print(f"Mission time completed ({elapsed_time:.1f} seconds)")
                        break
                    
                    # Execute box movement while keeping vertical motors running
                    if not self.box_movement():
                        break
                    
                    # Ensure vertical motors stay running at 70%
                    self.motor_control.down(70)
                
            except Exception as e:
                print(f"Error in mission thread: {e}")
            finally:
                try:
                    print("Mission complete, keeping vertical motors running...")
                    self.motor_control.down(70)  # Keep vertical motors running
                except:
                    pass

        # Clear any previous stop event and start new mission
        self.stop_event.clear()
        self.mission_thread = threading.Thread(target=mission)
        self.mission_thread.start()

    def stop(self):
        """Stop the mission and all motors"""
        print("Stopping autonomous mission...")
        self.stop_event.set()
        
        try:
            if hasattr(self, 'mission_thread') and self.mission_thread.is_alive():
                print("Waiting for mission thread to end...")
                self.mission_thread.join(timeout=2)
            
            print("Mission stopped, stopping all motors...")
            self.motor_control.stop()  # This will stop all motors completely
        except Exception as e:
            print(f"Error during stop: {e}")
            try:
                print("Emergency stop attempt...")
                self.motor_control.stop()
            except:
                print("Emergency stop failed!")
