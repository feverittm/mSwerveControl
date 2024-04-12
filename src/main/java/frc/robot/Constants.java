// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final double kTrackWidth = 0.5;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.7;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = false;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The SysId tool provides a convenient method for obtaining these values for your robot.
        public static final double ksVolts = 1;
        public static final double kvVoltSecondsPerMeter = 0.8;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;

        public static final double kMaxSpeedMetersPerSecond = 4.4;
    }

    public static final class ModuleConstants {

        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) kEncoderCPR;

        /** Constants that apply to the whole drive train. */
        public static final double TRACK_WIDTH = Units.inchesToMeters(24.0); // Width of the drivetrain measured from the middle of the wheels.
        public static final double WHEEL_BASE = Units.inchesToMeters(24.0); // Length of the drivetrain measured from the middle of the wheels.
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        // SDS Mk4i L2 Drive Ratio = 6.75:1, Angle Ratio = 150/7:1
        public static final double DRIVE_GEAR_RATIO = 6.75 / 1.0; // 6.75:1
        public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
        public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
        public static final double ANGLE_MOTOR_GEAR_RATIO = 12.8 / 1.0; // 12.8:1

        /* note that these angle constant refer to the CTRE absolute encoder */
        public static final double kAngleEncodeAnglePerRev = (Math.PI * 2);
        public static final double kAngleEncodeVelocityPerRev = DRIVE_ROTATIONS_TO_METERS / 60.0;

        /** Idle modes. */
        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

        /** Current limiting. */
        public static final int DRIVE_CURRENT_LIMIT = 35;
        public static final int ANGLE_CURRENT_LIMIT = 25;

        /** Module PID Kp constants */
        public static final double kPModuleTurningController = 0.20;
        public static final double kPModuleDriveController = 0.0020645;

        /**
         * Module Configurations:
         * CanCoder offset is a raw number (0-1), this is measured using the rev client (or the
         * value from the SmartDashboard).  If the code determines that a value has already 
         * been set on the SparkMax using the Rev Client, then use it instead.
         * 
         * The hardware settings have been copied from the python repository.
         */

         // Front Left Module
        public static final SwerveModuleConstants Front_Left_Configuration = new SwerveModuleConstants(
                1,
                8,
                1,
                false,
                true,
                false,
                2 * Math.PI,
                0
        );

        // Front Right
        public static final SwerveModuleConstants Front_Right_Configuration = new SwerveModuleConstants(
                2,
                6,
                7,
                false,
                true,
                false,
                2 * Math.PI,
                0
        );

        // Back Right
        public static final SwerveModuleConstants Back_Right_Configuration = new SwerveModuleConstants(
                3,
                4,
                5,
                false,
                true,
                false,
                2 * Math.PI,
                0
        );

        // Back Left
        public static final SwerveModuleConstants Back_Left_Configuration = new SwerveModuleConstants(
                4,
                2,
                3,
                false,
                true,
                false,
                2 * Math.PI,
                 0
        );
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int GYRO_RESET_BUTTON = XboxController.Button.kA.value;
        public static final int ENCODER_RESET_BUTTON = XboxController.Button.kB.value;

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.01;
        public static final double LEFT_Y_DEADBAND = 0.01;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
