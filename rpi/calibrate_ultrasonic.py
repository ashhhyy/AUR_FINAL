import time
import RPi.GPIO as GPIO

class UltrasonicSensorCalibrator:
    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def get_raw_distance(self):
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, GPIO.LOW)

        start_time = time.time()
        stop_time = time.time()

        # Add a short timeout to prevent infinite loops if sensor isn't responding
        timeout_start = time.time() + 0.1 # 100ms timeout for echo start
        while GPIO.input(self.echo_pin) == 0:
            start_time = time.time()
            if time.time() > timeout_start:
                # print("Timeout waiting for echo start") # Optional: uncomment for debugging
                return -1

        timeout_end = time.time() + 0.1 # 100ms timeout for echo end
        while GPIO.input(self.echo_pin) == 1:
            stop_time = time.time()
            if time.time() > timeout_end:
                # print("Timeout waiting for echo end") # Optional: uncomment for debugging
                return -1

        # Ensure stop_time is indeed after start_time, otherwise reading is invalid
        if stop_time <= start_time:
            # print("Invalid timing, stop time not after start time") # Optional: uncomment for debugging
            return -1

        time_elapsed = stop_time - start_time
        raw_distance = (time_elapsed * 34300) / 2
        return raw_distance

    # Removed the 'delay' parameter and the time.sleep(delay) call
    def calibrate(self, known_distances_cm, num_readings=5):
        offsets = []
        scales = []
        for i, known_distance in enumerate(known_distances_cm):
            # Removed the "Sending actual reading every X seconds" print
            readings = []
            print(f"Taking {num_readings} readings for known distance {known_distance} cm")
            for _ in range(num_readings):
                raw_distance = self.get_raw_distance()
                if raw_distance == -1:
                    print("Invalid reading, skipping")
                    continue
                print(f"Actual reading: {raw_distance:.2f} cm")
                readings.append(raw_distance)
                # Removed: time.sleep(delay)
                time.sleep(0.1) # Small delay between readings to allow sensor to settle, adjust as needed

            if not readings:
                print("No valid readings, skipping this distance")
                continue
            
            avg_raw = sum(readings) / len(readings)
            
            # Calibration logic: y = mx + c => known_distance = scale * avg_raw + offset
            # One common approach for linear calibration with two points:
            # Calculate offset for each point (offset = known - raw)
            # Calculate scale for each point (scale = known / raw)
            # Then average them. This assumes a simple linear model.
            
            # If your calibration needs to be more robust, consider
            # gathering data for at least two known distances and
            # performing a linear regression (e.g., using numpy.polyfit)
            # to find a single best-fit slope (scale) and intercept (offset).
            
            offset = known_distance - avg_raw
            scale = known_distance / avg_raw if avg_raw != 0 else 1 # Avoid division by zero
            
            offsets.append(offset)
            scales.append(scale)
            print(f"Calibration for distance {known_distance} cm: offset={offset:.2f}, scale={scale:.4f}")
        
        if offsets and scales:
            avg_offset = sum(offsets) / len(offsets)
            avg_scale = sum(scales) / len(scales)
            print(f"Average calibration results: offset={avg_offset:.2f}, scale={avg_scale:.4f}")
            return avg_offset, avg_scale
        else:
            print("Calibration failed: no valid data")
            return 0, 1 # Return default values if calibration fails

def main():
    sensors = [
        {'trigger_pin': 5, 'echo_pin': 25, 'name': 'sensor_1'},
        {'trigger_pin': 7, 'echo_pin': 8, 'name': 'sensor_2'},
        {'trigger_pin': 20, 'echo_pin': 21, 'name': 'sensor_3'}
    ]

    calibration_results = {}

    for sensor in sensors:
        print(f"\n--- Calibrating {sensor['name']} on trigger pin {sensor['trigger_pin']} and echo pin {sensor['echo_pin']} ---")
        calibrator = UltrasonicSensorCalibrator(sensor['trigger_pin'], sensor['echo_pin'])
        
        known_distances = []
        # Modified to prompt for known distances BEFORE taking all readings for that distance
        # This makes the calibration flow clearer: set object, then measure.
        print(f"Prepare to measure {sensor['name']}.")
        
        # It's better to ask for distinct known distances rather than 5 readings for 5 potentially
        # different unknown distances. Let's aim for 2-3 distinct known points for simple calibration.
        # For more accurate calibration, you'd typically have multiple known distances.
        
        # Changed the loop to ask for a known distance, then take readings for that distance.
        # This will simplify the user interaction.
        num_calibration_points = 3 # Example: calibrate at 3 different known distances
        for i in range(num_calibration_points):
            while True:
                try:
                    known_distance_input = float(input(f"Enter the ACTUAL known distance for calibration point {i+1} in cm (e.g., 10, 20, 30): "))
                    if known_distance_input <= 0:
                        print("Distance must be positive.")
                        continue
                    break
                except ValueError:
                    print("Invalid input. Please enter a numeric value.")
            known_distances.append(known_distance_input)
            
            # Provide instructions for taking readings at this known distance
            print(f"Now, place an object exactly {known_distances[-1]:.2f} cm from {sensor['name']} and press Enter to take {5} readings.")
            input("Press Enter when ready...") # Wait for user to set up
            
        offset, scale = calibrator.calibrate(known_distances, num_readings=5) # Take 5 readings per known distance
        calibration_results[sensor['name']] = {'offset': offset, 'scale': scale}
        
        # Clean up GPIO for this sensor before moving to the next
        GPIO.cleanup()

    print("\n--- All Calibration Results ---")
    for sensor_name, results in calibration_results.items():
        print(f"{sensor_name}: offset={results['offset']:.2f} cm, scale={results['scale']:.4f}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCalibration interrupted by user.")
    finally:
        GPIO.cleanup() # Ensure GPIO is cleaned up on exit