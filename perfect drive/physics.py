"""
Physics simulation support for swerve drive
"""
from pyfrc.physics.core import PhysicsInterface
from wpimath.system.plant import DCMotor
from wpimath.geometry import Pose2d, Rotation2d
from wpilib import RobotController, simulation, Timer, DriverStation  # Add Timer and DriverStation
import wpilib  # Add general wpilib import
import math
import constants

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface):
        self.physics_controller = physics_controller
        self.pose = Pose2d()
        # Add simulation timer
        self.match_timer = wpilib.Timer()
        self.match_timer.start()
        
        # Create PWM simulation objects
        self.pwm_sims = {
            constants.FRONT_LEFT_DRIVE: simulation.PWMSim(constants.FRONT_LEFT_DRIVE),
            constants.FRONT_LEFT_TURN: simulation.PWMSim(constants.FRONT_LEFT_TURN),
            constants.FRONT_RIGHT_DRIVE: simulation.PWMSim(constants.FRONT_RIGHT_DRIVE),
            constants.FRONT_RIGHT_TURN: simulation.PWMSim(constants.FRONT_RIGHT_TURN),
            constants.BACK_LEFT_DRIVE: simulation.PWMSim(constants.BACK_LEFT_DRIVE),
            constants.BACK_LEFT_TURN: simulation.PWMSim(constants.BACK_LEFT_TURN),
            constants.BACK_RIGHT_DRIVE: simulation.PWMSim(constants.BACK_RIGHT_DRIVE),
            constants.BACK_RIGHT_TURN: simulation.PWMSim(constants.BACK_RIGHT_TURN),
        }

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Updates the simulation with new robot position
        """
        try:
            # Silence joystick warnings
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)
            
            # Skip match time setting since it's not supported in simulation
            # Just use the timer for internal tracking
            match_time = self.match_timer.get()
            
            # Get motor values without inversion
            drive_fl = self.pwm_sims[constants.FRONT_LEFT_DRIVE].getSpeed()
            drive_fr = self.pwm_sims[constants.FRONT_RIGHT_DRIVE].getSpeed()
            drive_bl = self.pwm_sims[constants.BACK_LEFT_DRIVE].getSpeed()
            drive_br = self.pwm_sims[constants.BACK_RIGHT_DRIVE].getSpeed()
            
            # Calculate forward movement (no inversion needed)
            forward = (drive_fl + drive_fr + drive_bl + drive_br) / 4.0 * constants.MAX_SPEED
            strafe = 0  # Will be calculated from turn motor positions
            rotation = ((drive_fr + drive_br) - (drive_fl + drive_bl)) / 4.0 * constants.MAX_ANGULAR_SPEED
            
            # Calculate robot-relative movement
            current_rotation = self.pose.rotation()
            cos_angle = current_rotation.cos()
            sin_angle = current_rotation.sin()
            
            # Calculate final velocities
            vx = forward * cos_angle - strafe * sin_angle
            vy = forward * sin_angle + strafe * cos_angle
            
            # Update pose
            new_pose = Pose2d(
                self.pose.X() + vx * tm_diff,
                self.pose.Y() + vy * tm_diff,
                Rotation2d(self.pose.rotation().radians() + rotation * tm_diff)
            )
            self.pose = new_pose
            
            # Update the simulation
            self.physics_controller.field.setRobotPose(self.pose)
            
            # Debug output
            print(f"Motors - FL: {drive_fl:.2f}, FR: {drive_fr:.2f}, BL: {drive_bl:.2f}, BR: {drive_br:.2f}")
            print(f"Robot Pose - X: {self.pose.X():.2f}, Y: {self.pose.Y():.2f}, Rot: {self.pose.rotation().degrees():.2f}°")
            
        except Exception as e:
            print(f"Physics Update Error: {str(e)}")
            import traceback
            traceback.print_exc()
