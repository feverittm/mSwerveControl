package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class ConfigSwerveNeo {
    private SwerveModuleConstants m_module_constants;
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turningMotor;
    private RelativeEncoder m_driveMotorEncoder;
    private RelativeEncoder m_turningMotorEncoder;
    private SparkAbsoluteEncoder m_angleEncoder;

    private PIDController m_drivePIDController;

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private ProfiledPIDController m_turningPIDController;

    // For tuning only.  Use a simple pid P-Only controller to get the value for kP.  
    // Then we can work on Ka and Ks (trapezoidal constraints) 
    private PIDController m_simpleTurningPIDController;

    public void configDevices (SwerveModuleConstants module_constants, 
        PIDController drivePIDController, ProfiledPIDController turningPIDController) {
        
        this.m_module_constants = module_constants;
        this.m_drivePIDController = drivePIDController;
        this.m_turningPIDController = turningPIDController;

        // Drive motor configuration:
        // - L2 NEO Motor connected to SParkMax
        // - SDS4i model L2 has a drive motor to wheel ratio of 6.75:1
        // and an adjusted speed of 14.5 ft/sec with the NEO motors
        // - Outside wheel diameter = 4in
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.clearFaults();
        
        if (m_driveMotor.setIdleMode(Constants.ModuleConstants.DRIVE_IDLE_MODE) != REVLibError.kOk) {
        SmartDashboard.putString("Drive Motor Idle Mode", "Error");
        }
        m_driveMotor.setInverted(m_module_constants.driveMotorReversed);
        m_driveMotor.setSmartCurrentLimit(Constants.ModuleConstants.DRIVE_CURRENT_LIMIT);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_driveMotorEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);

        // Angle motor configuration.
        // Neo Motor connected to SParkMax (all turn motors are reversed in the SDS 4i)
        // The steering gear ration (from turning motor to wheel) is 150/7:1 = 21.43
        m_turningMotor.restoreFactoryDefaults();
        m_turningMotor.clearFaults();
        if (m_turningMotor.setIdleMode(Constants.ModuleConstants.ANGLE_IDLE_MODE) != REVLibError.kOk) {
        SmartDashboard.putString("Turn Motor Idle Mode", "Error");
        }
        m_turningMotor.setInverted(module_constants.angleMotorReversed);
        m_turningMotor.setSmartCurrentLimit(Constants.ModuleConstants.ANGLE_CURRENT_LIMIT);

        // this is the relative encoder included in the NEO motor.
        m_turningMotorEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);

        /**
         * CTRE Mag Encoder connected to the SparkMAX Absolute/Analog/PWM Duty Cycle
        * input
        * Native will ready 0.0 -> 1.0 for each revolution.
        */
        m_angleEncoder.setZeroOffset(module_constants.angleEncoderOffset);
        m_angleEncoder.setInverted(module_constants.angleEncoderReversed);
        //m_angleEncoder.setPositionConversionFactor(2*Math.PI);
        m_angleEncoder.setAverageDepth(8);
        m_angleEncoder.setPositionConversionFactor(module_constants.angleEncoderConversionFactor);
        
        /**
         * Make PID continuous around the 180degree point of the rotation
        *            0
        *      PI/2  +  -PI/2
        *          PI/-PI
        * Note that + angle goes CCW from the zero point of the encoder.
        */
        m_simpleTurningPIDController = new PIDController(
        ModuleConstants.kPModuleTurningController,
        0,
        0);

        m_turningPIDController = new ProfiledPIDController(
            ModuleConstants.kPModuleTurningController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_turningPIDController.setTolerance(0.001);
        m_simpleTurningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

        m_drivePIDController.setTolerance(0.1, 0.1);
    }
}
