"""
Configuration Constants - These values define how the robot behaves

KEY SECTIONS:
1. Controller Settings - Which port the joystick is plugged into
2. Motor IDs - Which motors are connected to which ports
3. Robot Measurements - Physical size of the robot
4. Speed Limits - How fast the robot can move
5. Control Tuning - How precisely the wheels maintain their direction
"""
import wpilib
# SECTION 1: Controller Settings
# Which port number the driver's controller is plugged into
DRIVER_CONTROLLER = 1

# SECTION 2: Motor and Sensor IDs
# Updated motor configuration
FRONT_LEFT_DRIVE = 1    
FRONT_LEFT_TURN = 2     
FRONT_LEFT_ENCODER = 0  

FRONT_RIGHT_DRIVE = 3   
FRONT_RIGHT_TURN = 4    
FRONT_RIGHT_ENCODER = 1 

BACK_LEFT_DRIVE = 5     
BACK_LEFT_TURN = 6      
BACK_LEFT_ENCODER = 2   

BACK_RIGHT_DRIVE = 7    
BACK_RIGHT_TURN = 8     
BACK_RIGHT_ENCODER = 3  

# SECTION 3: Robot Physical Characteristics
# These measurements affect how the robot moves
WHEELBASE = 0.5   # Distance between front and back wheels (meters)
TRACKWIDTH = 0.5  # Distance between left and right wheels (meters)

# SECTION 4: Speed Limits
# These speed limits determine how fast the robot can move
MAX_SPEED = 0.5         # Increased back for better response
MAX_ANGULAR_SPEED = 0.3 # Increased for better turning
RAMP_RATE = 0.1        # Increased for less stuttering

# SECTION 5: PID Control Values
# These values control how accurately the wheels maintain their direction
TURNING_KP = 5.0  # Reduced for smoother control
TURNING_KI = 0.01  # Small integral for steady-state
TURNING_KD = 0.1  # Reduced for less oscillation
TURNING_MAX_RATE = 0.3  # Max turning speed (rotations per second)

# SECTION 6: Autonomous Settings
AUTO_TIME = 1.5  # How long autonomous runs (in seconds)

# SECTION 7: Elevator and Climber Settings
ELEVATOR_KP = 0.5
ELEVATOR_KI = 0.0
ELEVATOR_KD = 0.1
ELEVATOR_MOTOR = 9
CLIMBER_MOTOR = 10
ELEVATOR_UP_SPEED = 0.9      # 90% speed up
ELEVATOR_DOWN_SPEED = -0.7   # 70% speed down
CLIMBER_UP_SPEED = 1.0     # Full speed up
CLIMBER_DOWN_SPEED = -0.8   # 80% speed down
ELEVATOR_DEADBAND = 0.05    # Reduced deadband for more responsive control
ELEVATOR_MAX_HEIGHT = 1.0  # meters
ELEVATOR_MIN_HEIGHT = 0.0  # meters

# SECTION 8: Drive Control
ACCELERATION_FACTOR = 0.6    # Increased for better response
DECELERATION_FACTOR = 0.2   # Increased for less stuttering
INPUT_SMOOTHING = 0.3       # Increased for smoother control