#!/bin/bash

echo "Setting up Autonomous Underwater Robot..."

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo "Warning: This script is designed for Raspberry Pi. Some features may not work on other systems."
fi

# Update package lists and install system dependencies for Python builds
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y build-essential python3-dev python3-pip pkg-config

# Install Node.js and npm if not present (optional, can be removed if not using web-dashboard)
echo "Checking Node.js and npm installation..."
if ! command -v node &> /dev/null; then
    echo "Installing Node.js and npm..."
    curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
    sudo apt-get install -y nodejs
    echo "Node.js version: $(node --version)"
    echo "npm version: $(npm --version)"
else
    echo "Node.js already installed: $(node --version)"
    echo "npm already installed: $(npm --version)"
fi

# Create virtual environment and install Python dependencies
echo "Setting up Python environment..."
if ! python3 -m venv venv; then
    echo "Error: Failed to create virtual environment"
    exit 1
fi

source venv/bin/activate || { echo "Error: Failed to activate virtual environment"; exit 1; }
pip install --upgrade pip setuptools wheel

# Install Python packages with better error handling
echo "Installing Python packages..."
if ! pip install -r requirements.txt; then
    echo "Warning: Some Python packages failed to install. Trying alternative installation..."
    # Try installing packages individually to identify problematic ones
    pip install Flask Flask-CORS
    pip install RPi.GPIO || echo "Warning: RPi.GPIO installation failed (normal on non-Pi systems)"
    pip install adafruit-circuitpython-mpu6050 adafruit-blinka || echo "Warning: Adafruit libraries installation failed"
    pip install Pillow numpy
fi

# Setup web dashboard
echo "Setting up web dashboard..."
if [ -d "web-dashboard" ]; then
    cd web-dashboard
    if ! npm install; then
        echo "Error: npm install failed. Please check your Node.js installation."
        exit 1
    fi
    cd ..
else
    echo "Warning: web-dashboard directory not found, skipping frontend setup."
fi

# Create necessary directories
echo "Creating necessary directories..."
mkdir -p images
mkdir -p logs

# Configure permissions for hardware access
echo "Configuring hardware permissions..."
if grep -q "Raspberry Pi" /proc/cpuinfo; then
    # Enable I2C if not already enabled
    if ! grep -q "^i2c-dev" /etc/modules; then
        echo "i2c-dev" | sudo tee -a /etc/modules
    fi
    
    # Enable SPI if not already enabled (optional for this project)
    if ! grep -q "^spi-dev" /etc/modules; then
        echo "spi-dev" | sudo tee -a /etc/modules
    fi
    
    # Add user to required groups
    sudo usermod -a -G gpio,i2c,spi $USER
    
    # Reload modules with error handling
    echo "Loading I2C module..."
    sudo modprobe i2c-dev || echo "Warning: i2c-dev module load failed"
    
    echo "Loading SPI module..."
    sudo modprobe spi-dev || echo "Warning: spi-dev module load failed (this may be normal)"
fi

# Create systemd service for auto-start (only on Raspberry Pi)
if grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo "Creating systemd service..."
    SERVICE_PATH="/etc/systemd/system/underwater-robot.service"
    VENV_PATH="$(pwd)/venv"
    APP_PATH="$(pwd)/rpi/app.py"
    USER_NAME="$USER"
    sudo tee $SERVICE_PATH > /dev/null << EOL
[Unit]
Description=Autonomous Underwater Robot Service
After=network.target

[Service]
ExecStart=$VENV_PATH/bin/python $APP_PATH
WorkingDirectory=$(pwd)
User=$USER_NAME
Environment=PYTHONPATH=$(pwd)
Restart=always

[Install]
WantedBy=multi-user.target
EOL

    # Enable and start service
    sudo systemctl enable underwater-robot
    sudo systemctl start underwater-robot
fi

echo "Setup complete!"
echo "To start the application:"
echo "1. Activate Python environment: source venv/bin/activate"
echo "2. Run Flask API: python rpi/app.py"
