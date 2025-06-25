"""
Flask web app for control and dashboard.
Provides start/stop motion control and displays latest images.
"""

from flask import Flask, render_template, jsonify, request, send_from_directory
from flask_cors import CORS
import threading
import time
import os
import glob
import logging
import platform

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Force use of real hardware modules for actual drone control
logger.info("Forcing use of real hardware modules for drone control")

try:
    from sensors import UltrasonicSensor
except ImportError as e:
    logger.warning(f"Failed to import sensors module: {e}")
    from sensors_mock import UltrasonicSensor

from autonomous_logic import AutonomousLogic

app = Flask(__name__)
CORS(app)  # Enable CORS for web dashboard

motion_running = False
motion_thread = None

# Configuration
import os

UPLOAD_DIRECTORY = "./uploaded_images"
if not os.path.exists(UPLOAD_DIRECTORY):
    os.makedirs(UPLOAD_DIRECTORY)

IMAGE_DIRECTORY = UPLOAD_DIRECTORY  # Use uploaded images directory
MAX_IMAGES = 5

from motor_control import MotorControl
from sensors import UltrasonicSensor
from autonomous_logic import AutonomousLogic

class DummySensor:
    def get_distance(self):
        return 1000.0  # Large distance to simulate no obstacle

motor_control = MotorControl()
front_sensor = DummySensor()  # Disabled front sensor
back_sensor = UltrasonicSensor(trigger_pin=7, echo_pin=8)
bottom_sensor = UltrasonicSensor(trigger_pin=20, echo_pin=21)

try:
    autonomous_logic = AutonomousLogic(motor_control, front_sensor, back_sensor, bottom_sensor)
    logger.info("AutonomousLogic initialized successfully")
except Exception as e:
    logger.error(f"Failed to initialize AutonomousLogic: {e}")
    autonomous_logic = None

def autonomous_run():
    """Run autonomous logic in a separate thread"""
    global motion_running
    try:
        if autonomous_logic:
            autonomous_logic.run()
        while motion_running:
            time.sleep(1)
    except Exception as e:
        logger.error(f"Error in autonomous run: {e}")
        motion_running = False

def get_latest_images():
    """Get the latest images from ESP32-CAM"""
    try:
        if not os.path.exists(IMAGE_DIRECTORY):
            logger.warning(f"Image directory {IMAGE_DIRECTORY} does not exist")
            return []
        
        # Get all jpg files and sort by modification time
        image_files = glob.glob(os.path.join(IMAGE_DIRECTORY, "*.jpg"))
        image_files.sort(key=os.path.getmtime, reverse=True)
        
        # Return the latest MAX_IMAGES files
        latest_images = image_files[:MAX_IMAGES]
        
        # Convert to relative paths for web serving
        return [os.path.basename(img) for img in latest_images]
    except Exception as e:
        logger.error(f"Error getting latest images: {e}")
        return []

@app.route('/')
def index():
    """Render dashboard page"""
    return render_template('index.html')

@app.route('/start', methods=['POST'])
def start_motion():
    """Start autonomous motion control"""
    global motion_running, motion_thread
    try:
        if not motion_running and autonomous_logic:
            motion_running = True
            motion_thread = threading.Thread(target=autonomous_run)
            motion_thread.daemon = True
            motion_thread.start()
            logger.info("Motion control started")
            return jsonify({'status': 'started', 'success': True})
        elif not autonomous_logic:
            logger.error("Hardware not initialized - cannot start motion")
            return jsonify({'status': 'error', 'message': 'Hardware not initialized', 'success': False}), 500
        else:
            logger.warning("Motion control already running")
            return jsonify({'status': 'already_running', 'success': True})
    except Exception as e:
        logger.error(f"Error starting motion: {e}", exc_info=True)
        return jsonify({'status': 'error', 'message': str(e), 'success': False}), 500

@app.route('/stop', methods=['POST'])
def stop_motion():
    """Stop autonomous motion control"""
    global motion_running, motion_thread
    try:
        if motion_running:
            motion_running = False
            if autonomous_logic:
                autonomous_logic.stop()  # This will handle stopping motors
            if motion_thread and motion_thread.is_alive():
                motion_thread.join(timeout=5)
            logger.info("Motion control stopped")
            return jsonify({'status': 'stopped', 'success': True})
        else:
            return jsonify({'status': 'already_stopped', 'success': True})
    except Exception as e:
        logger.error(f"Error stopping motion: {e}")
        return jsonify({'status': 'error', 'message': str(e), 'success': False}), 500

@app.route('/status')
def status():
    """Get current robot status"""
    try:
        return jsonify({
            'motion_running': motion_running,
            'hardware_initialized': autonomous_logic is not None,
            'success': True
        })
    except Exception as e:
        logger.error(f"Error getting status: {e}")
        return jsonify({'status': 'error', 'message': str(e), 'success': False}), 500

@app.route('/images')
def images():
    """Get latest captured images"""
    try:
        latest_images = get_latest_images()
        if not latest_images:
            logger.warning("No images found in IMAGE_DIRECTORY or directory missing")
        return jsonify({
            'images': latest_images,
            'count': len(latest_images),
            'success': True
        })
    except Exception as e:
        logger.error(f"Error getting images: {e}")
        return jsonify({'images': [], 'count': 0, 'success': False, 'message': str(e)})

@app.route('/images/<filename>')
def serve_image(filename):
    """Serve image files from IMAGE_DIRECTORY"""
    try:
        return send_from_directory(IMAGE_DIRECTORY, filename)
    except Exception as e:
        logger.error(f"Error serving image {filename}: {e}")
        return jsonify({'error': 'Image not found', 'success': False}), 404

from werkzeug.utils import secure_filename
from flask import request

@app.route('/upload_image', methods=['POST'])
def upload_image():
    """Endpoint to receive image uploads from ESP32"""
    if 'image' not in request.files:
        return jsonify({'success': False, 'message': 'No image part in the request'}), 400
    file = request.files['image']
    if file.filename == '':
        return jsonify({'success': False, 'message': 'No selected file'}), 400
    filename = secure_filename(file.filename)
    save_path = os.path.join(IMAGE_DIRECTORY, filename)
    try:
        file.save(save_path)
        logger.info(f"Received and saved image: {filename}")
        return jsonify({'success': True, 'message': f'Image {filename} uploaded successfully'})
    except Exception as e:
        logger.error(f"Failed to save uploaded image {filename}: {e}")
        return jsonify({'success': False, 'message': str(e)}), 500

@app.errorhandler(404)
def not_found(error):
    return jsonify({'error': 'Not found', 'success': False}), 404

@app.errorhandler(500)
def internal_error(error):
    return jsonify({'error': 'Internal server error', 'success': False}), 500

if __name__ == '__main__':
    logger.info("Starting Flask application...")
    app.run(host='0.0.0.0', port=5000, debug=True)
