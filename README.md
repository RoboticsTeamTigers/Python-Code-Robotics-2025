# Python-Code-Robotics-2025
Python Code

First, install the necessary tools:
pip install robotpy
pip install robotpy[all]
pip install robotpy robotpy-wpilib robotpy-wpimath
pip install pygame
python -m robotpy installer download-python

# First, install Python on the roboRIO (only needed once)
python -m robotpy deploy-python

# Then deploy your robot code
python -m robotpy deploy

# If you need to see detailed output, add --verbose
python -m robotpy deploy --verbose

Troubleshooting:
# Test connection to roboRIO
ping roborio-TEAM-frc.local

# Show deployment status
python -m robotpy deploy-info

# Test connection to roboRIO
ping roborio-TEAM-frc.local

# Show deployment status
python -m robotpy deploy-info

# Try running the simulator
python -m robotpy sim
# MAKE SURE YOU HAVE ROBOT PY REV INSTALLED
python -m pip install -U robotpy-rev

# Install python onto robot
python -m pip install robotpy[all]==2025.3.1.1
python -m pip install robotpy-rev==2025.0.2
python -m pip install phoenix6==25.1.0

# Connect to robot and install packages
python -m robotpy deploy-info
python -m robotpy sync
python -m robotpy deploy
