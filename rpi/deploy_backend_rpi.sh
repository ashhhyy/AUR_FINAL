#!/bin/bash

# Script to deploy and run backend on Raspberry Pi

# Update and install python3-venv if not installed
sudo apt update
sudo apt install -y python3-venv git

# Clone or update the project repository
if [ ! -d "new_aur" ]; then
  git clone https://github.com/your-repo/new_aur.git
else
  cd new_aur
  git pull
  cd ..
fi

cd new_aur

# Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate

# Install python dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Verify IMAGE_DIRECTORY path in rpi/app.py
echo "Please verify IMAGE_DIRECTORY path in rpi/app.py matches your ESP32-CAM image storage location."

# Run the backend Flask app
echo "Starting backend Flask app..."
python rpi/app.py
