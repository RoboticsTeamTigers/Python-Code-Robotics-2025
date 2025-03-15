"""
Swerve Module Control

This file contains the code for controlling a single swerve module.
A swerve module consists of:
1. Drive Motor - Controls wheel speed
2. Turn Motor - Controls wheel direction
3. Encoder - Measures wheel direction

The module uses PID control to accurately maintain wheel direction.
"""

import wpilib
from wpimath import geometry
from wpimath.controller import PIDController  # Updated import
import math
import constants

class SwerveModule:
    def __init__(self, driveMotorChannel: int, turningMotorChannel: int, 
                 encoderChannel: int, name: str):
        """
        Initialize a single swerve module (one corner of the robot)
        Each module has:
        - Drive motor: Controls wheel speed (forward/backward)
        - Turn motor: Controls wheel angle/direction
        - Encoder: Measures the wheel's current angle
        """
        try:
            print(f"Initializing {name} module...")
            self.name = name
            
            # Initialize motors
            self.driveMotor = wpilib.PWMSparkMax(driveMotorChannel)
            self.turningMotor = wpilib.PWMSparkMax(turningMotorChannel)
            self.encoder = wpilib.AnalogEncoder(encoderChannel)
            
            # Set all drive motors to non-inverted by default
            self.driveMotor.setInverted(False)
            self.turningMotor.setInverted(False)
            
            # Create turning PID controller with improved values
            self.turn_pid = PIDController(
                constants.TURNING_KP,
                constants.TURNING_KI,
                constants.TURNING_KD
            )
            self.turn_pid.enableContinuousInput(-math.pi, math.pi)
            self.turn_pid.setTolerance(0.01)  # 0.01 radians tolerance
            
            # Add rate limiting for smoother turns
            self.previous_angle = 0
            
            self.stop()
            print(f"{name} module initialization complete")
            
        except Exception as e:
            print(f"Failed to initialize {name} module: {e}")
            raise

    def stop(self):
        """Emergency stop for this module"""
        try:
            self.driveMotor.set(0)
            self.turningMotor.set(0)
        except Exception as e:
            print(f"Error stopping {self.name} module: {e}")

    def setDriveInverted(self, inverted: bool):
        """Set whether the drive motor is inverted"""
        self.drive_inverted = inverted
        self.driveMotor.setInverted(inverted)
        print(f"{self.name} drive motor inversion set to {inverted}")

    def setTurnInverted(self, inverted: bool):
        """Set whether the turn motor is inverted"""
        self.turn_inverted = inverted
        self.turningMotor.setInverted(inverted)
        print(f"{self.name} turn motor inversion set to {inverted}")

    def setDesiredState(self, state):
        try:
            # Convert speed to motor power (-1 to 1)
            drive_power = state.speed / constants.MAX_SPEED
            
            # Get current angle and target angle
            current_angle = self.encoder.get() * 2 * math.pi
            target_angle = state.angle.radians()
            
            # Rate limit the angle change
            angle_change = target_angle - self.previous_angle
            angle_change = min(max(angle_change, -constants.TURNING_MAX_RATE), 
                             constants.TURNING_MAX_RATE)
            target_angle = self.previous_angle + angle_change
            self.previous_angle = target_angle
            
            # Calculate turn power using PID
            turn_power = self.turn_pid.calculate(current_angle, target_angle)
            turn_power = min(max(turn_power, -1), 1)  # Limit power to [-1, 1]
            
            # Set motor powers
            self.driveMotor.set(drive_power)
            self.turningMotor.set(turn_power)
            
        except Exception as e:
            print(f"Error setting {self.name} state: {e}")
            self.stop()
            raise
