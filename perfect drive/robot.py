"""
Main Robot Code for Swerve Drive

This file contains the main robot class that controls a swerve drive system.
A swerve drive allows the robot to move in any direction and rotate independently.

Key Components:
- 4 Swerve Modules (each with drive and turning motors)
- Xbox Controller for input
- Kinematics for calculating wheel speeds and angles
"""

import wpilib
import wpilib.drive
from wpimath import kinematics
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds  # Add this import
from swervemodule import SwerveModule
import constants
import traceback  # Add this import

class MyRobot(wpilib.TimedRobot):
    """
    Main robot class that handles initialization and periodic control
    TimedRobot runs the robot code in timed loops (typically 50 times per second)
    """
    
    def robotInit(self):
        try:
            print("Starting robot initialization...")
            
            # Initialize controller first
            self.controller = wpilib.Joystick(constants.DRIVER_CONTROLLER)
            print("Controller initialized")
            
            # Initialize gyro
            self.gyro = wpilib.ADXRS450_Gyro()
            self.gyro.calibrate()
            print("Gyro initialized")
            
            # Initialize one module at a time with verification
            print("Initializing swerve modules one at a time...")
            self.initializeSwerveModules()
            print("All modules initialized")
            
            self.initializeKinematics()
            print("Kinematics initialized")
            
            # Initialize elevator motor
            self.elevator_motor = wpilib.PWMSparkMax(constants.ELEVATOR_MOTOR)
            self.elevator_motor.setInverted(False)  # Adjust if needed
            print("Elevator motor initialized")
            
            # Initialize elevator PID
            self.elevator_pid = wpilib.PIDController(
                constants.ELEVATOR_KP,
                constants.ELEVATOR_KI,
                constants.ELEVATOR_KD
            )
            self.elevator_setpoint = 0
            
            # Add watchdog
            self.watchdog = wpilib.Watchdog(0.5, self._watchdogCallback)
            self.watchdog.enable()
            
            # Initialize match time
            self.match_time = wpilib.DriverStation.getMatchTime()
            wpilib.DriverStation.refreshData()  # Refresh driver station data
            print(f"Match time initialized: {self.match_time}")
            
            # Add field relative state tracking
            self.field_relative = True
            self.robot_rotation = Rotation2d()  # Track robot's rotation
            
            print("Field relative control initialized")
            
        except Exception as e:
            print(f"CRITICAL ERROR in robotInit: {e}")
            traceback.print_exc()
            self._emergencyStop()

    def _watchdogCallback(self):
        """Called when watchdog times out"""
        print("Watchdog timeout - Check for stuck loops or motor issues")

    def _emergencyStop(self):
        """Safe shutdown of all systems"""
        try:
            if hasattr(self, 'modules'):
                for module in self.modules:
                    try:
                        module.stop()
                    except:
                        pass
        except:
            pass
        print("Emergency stop triggered")

    def initializeSwerveModules(self):
        """Initialize all swerve modules with error checking"""
        self.modules = []
        
        # Added inversion flags for each module (drive_inverted, turn_inverted)
        module_configs = [
            ("Front Left", constants.FRONT_LEFT_DRIVE, constants.FRONT_LEFT_TURN, constants.FRONT_LEFT_ENCODER, True, False),
            ("Front Right", constants.FRONT_RIGHT_DRIVE, constants.FRONT_RIGHT_TURN, constants.FRONT_RIGHT_ENCODER, True, False),
            ("Back Left", constants.BACK_LEFT_DRIVE, constants.BACK_LEFT_TURN, constants.BACK_LEFT_ENCODER, True, False),
            ("Back Right", constants.BACK_RIGHT_DRIVE, constants.BACK_RIGHT_TURN, constants.BACK_RIGHT_ENCODER, True, False),
        ]
        
        for name, drive, turn, encoder, drive_inverted, turn_inverted in module_configs:
            try:
                module = SwerveModule(drive, turn, encoder, name)
                module.setDriveInverted(drive_inverted)
                module.setTurnInverted(turn_inverted)
                self.modules.append(module)
                print(f"Successfully initialized {name} module")
            except Exception as e:
                print(f"Failed to initialize {name} module: {e}")
                raise
        
        # Assign modules to their positions
        [self.frontLeft, self.frontRight, 
         self.backLeft, self.backRight] = self.modules

    def initializeKinematics(self):
        """Initialize kinematics with proper error checking"""
        self.kinematics = kinematics.SwerveDrive4Kinematics(
            Translation2d(constants.WHEELBASE / 2, constants.TRACKWIDTH / 2),
            Translation2d(constants.WHEELBASE / 2, -constants.TRACKWIDTH / 2),
            Translation2d(-constants.WHEELBASE / 2, constants.TRACKWIDTH / 2),
            Translation2d(-constants.WHEELBASE / 2, -constants.TRACKWIDTH / 2)
        )

    def disabledInit(self):
        print("Robot disabled!")

    def teleopInit(self):
        """Called when teleop starts"""
        print("Teleop started!")
        
    def robotPeriodic(self):
        """Runs in all modes"""
        try:
            # Just use the timer value directly for match time display
            if hasattr(self, 'match_timer'):
                match_time = self.match_timer.get()
            else:
                match_time = 0
            
            wpilib.SmartDashboard.putString("Robot Mode", self.getCurrentMode())
            wpilib.SmartDashboard.putBoolean("Robot Enabled", self.isEnabled())
            wpilib.SmartDashboard.putNumber("Match Time", match_time)
            
        except Exception as e:
            print(f"Error in robotPeriodic: {e}")
            traceback.print_exc()

    def applyDriveCurve(self, input):
        """Apply cube root curve for smoother acceleration"""
        sign = 1 if input >= 0 else -1
        return sign * (abs(input) ** (1/3)) * constants.ACCELERATION_FACTOR

    def teleopPeriodic(self):
        """
        This is the main control loop where joystick inputs are converted to robot movement
        Direction Guide:
            - Forward/Back: Y-axis of joystick (push forward = robot moves forward)
            - Left/Right: X-axis of joystick (push right = robot moves right)
            - Rotation: Z-axis/twist of joystick (twist right = robot turns clockwise)
        """
        try:
            # Get joystick inputs with curve
            forward = self.applyDriveCurve(-self.controller.getRawAxis(1))  # Left Y
            strafe = self.applyDriveCurve(-self.controller.getRawAxis(0))   # Left X
            rotation = self.applyDriveCurve(-self.controller.getRawAxis(4)) # Right X

            # Apply deadband
            forward = self.applyDeadband(forward)
            strafe = self.applyDeadband(strafe)
            rotation = self.applyDeadband(rotation)

            # Get field-relative heading from gyro
            heading = Rotation2d.fromDegrees(-self.gyro.getAngle())

            # Create field-relative chassis speeds
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                forward * constants.MAX_SPEED,
                strafe * constants.MAX_SPEED,
                rotation * constants.MAX_ANGULAR_SPEED,
                heading
            )

            # Convert to module states and normalize
            states = self.kinematics.toSwerveModuleStates(chassis_speeds)
            kinematics.SwerveDrive4Kinematics.normalizeWheelSpeeds(states, constants.MAX_SPEED)

            # Update modules
            for i, module in enumerate(self.modules):
                module.setDesiredState(states[i])

            # Elevator Control using triggers
            right_trigger = self.controller.getRawAxis(3)  # Up
            left_trigger = self.controller.getRawAxis(2)   # Down
            
            elevator_power = 0
            if right_trigger > constants.ELEVATOR_DEADBAND:
                elevator_power = right_trigger * constants.ELEVATOR_UP_SPEED
            elif left_trigger > constants.ELEVATOR_DEADBAND:
                elevator_power = -left_trigger * abs(constants.ELEVATOR_DOWN_SPEED)
            
            self.elevator_motor.set(elevator_power)

            # Debug output
            wpilib.SmartDashboard.putNumber("Robot Heading", heading.degrees())
            wpilib.SmartDashboard.putNumber("Elevator Power", elevator_power)
            wpilib.SmartDashboard.putNumber("Right Trigger", right_trigger)
            wpilib.SmartDashboard.putNumber("Left Trigger", left_trigger)

        except Exception as e:
            print(f"Error in teleopPeriodic: {e}")
            traceback.print_exc()
            self._emergencyStop()
    
    def applyDeadband(self, value, deadband: float = 0.05):  # Reduced from 0.1 to 0.05
        """Apply deadband to joystick inputs"""
        if abs(value) < deadband:
            return 0
        return value

    def getCurrentMode(self):
        if self.isDisabled():
            return "Disabled"
        elif self.isAutonomous():
            return "Autonomous"
        elif self.isTeleop():
            return "Teleop"
        elif self.isTest():
            return "Test"
        else:
            return "Unknown"

    def autonomousInit(self):
        """Called when autonomous mode starts"""
        print("Autonomous started!")
        # Create and start the timer
        self.auto_timer = wpilib.Timer()
        self.auto_timer.start()

    def autonomousPeriodic(self):
        """
        Runs periodically during autonomous mode
        Will run for AUTO_TIME seconds then stop
        """
        try:
            # Check if we should still be running
            if self.auto_timer.get() < constants.AUTO_TIME:
                # Example: Drive forward at 20% speed during auto
                chassis_speeds = ChassisSpeeds(
                    0.2 * constants.MAX_SPEED,  # Forward at 20% speed
                    0.0,                        # No sideways movement
                    0.0                         # No rotation
                )
                
                # Convert to wheel commands
                states = self.kinematics.toSwerveModuleStates(chassis_speeds)
                
                # Command the wheels
                for i, module in enumerate(self.modules):
                    module.setDesiredState(states[i])
            else:
                # After AUTO_TIME seconds, stop all modules
                for module in self.modules:
                    module.stop()
                    
        except Exception as e:
            print(f"Error in autonomousPeriodic: {e}")
            traceback.print_exc()
            self._emergencyStop()

# Start the robot code when this file is run
if __name__ == "__main__":
    import robotpy
    robotpy.run(MyRobot)
