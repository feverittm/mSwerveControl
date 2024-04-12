// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.LinearMap;
import frc.robot.utils.SwerveModuleConstants;

public class SwerveModule {
  private SwerveModuleConstants module_constants;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveMotorEncoder;
  private final RelativeEncoder m_turningMotorEncoder;
  private final SparkAbsoluteEncoder m_angleEncoder;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  // For tuning only.  Use a simple pid P-Only controller to get the value for kP.  Then we can work on Ka and Ks (trapezoidal constraints) 
  private final PIDController m_simpleTurningPIDController = new PIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0);

  /**
   * Constructs a SwerveModule.
   *
   * @param module_constants The module constants (i.e. CAN IDs) for this specific
   *                         module
   */
  public SwerveModule(SwerveModuleConstants module_constants) {
    this.module_constants = module_constants;

    m_driveMotor = new CANSparkMax(module_constants.driveMotorID, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(module_constants.angleMotorID, MotorType.kBrushless);

    m_driveMotorEncoder = m_driveMotor.getEncoder();
    m_turningMotorEncoder = m_turningMotor.getEncoder();

    m_angleEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);

    configureDevices();
  }

  /**
   * Get the raw value from the absolute encoder on the SparkMax
   * 
   * @return raw angle (0.0->1.0)
   */
  public double getRawAngle() {
    return m_angleEncoder.getPosition();
  }

  public double getAbsoluteOffset() {
    return m_angleEncoder.getZeroOffset();
  }

  public boolean getAbsoluteEncoderInversion() {
    return m_angleEncoder.getInverted();
  }

  /**
   * Return the rotation vector for the absolute module angular position
   * 
   * @return angle vector mapped to the expected -pi->+pi range
   */
  public Rotation2d getAngle() {
    double raw_angle = getRawAngle();
    double mapped = LinearMap.map(raw_angle, 0.0, 1.0, -Math.PI, Math.PI);
    SmartDashboard.putNumber("Mapped Raw Module Angle", mapped);
    Rotation2d rot = new Rotation2d(mapped);
    return rot;
  }

  /**
   * Get the Drive motor encoder position
   * 
   * @return the drive encoder position
   */
  public double getDriveEncoderPosition() {
    return m_driveMotorEncoder.getPosition();
  }

  /**
   * Get the drive wheel velocity
   * 
   * @return drive encoder velocity
   */
  public double getDriveEncoderVelocity() {
    return m_driveMotorEncoder.getVelocity();
  }

  /**
   * Returns the current state (velocity/angle) of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double velocity = getDriveEncoderVelocity();
    return new SwerveModuleState(velocity, getAngle());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    double distance = getDriveEncoderPosition();
    Rotation2d rot = getAngle();
    return new SwerveModulePosition(distance, rot);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    //SwerveModuleState state = SwerveModuleState.optimize(desiredState, getState().angle);
    SwerveModuleState state = desiredState;

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveMotorEncoder.getVelocity(),
        state.speedMetersPerSecond);
    SmartDashboard.putNumber("state Mps", state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput_trap = m_turningPIDController.calculate(m_angleEncoder.getPosition(), state.angle.getRadians());
    final double turnOutput = m_simpleTurningPIDController.calculate(m_angleEncoder.getPosition(), state.angle.getRadians());

    SmartDashboard.putNumber("State/setpoint", m_turningPIDController.getSetpoint().position);
    SmartDashboard.putNumber("State/driveOutput", driveOutput);
    SmartDashboard.putNumber("State/turnOutput", turnOutput);
    SmartDashboard.putNumber("State/Trapezoidal turnOutput", turnOutput_trap);
    SmartDashboard.putNumber("Module offset", m_angleEncoder.getZeroOffset());



    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotorEncoder.setPosition(0.0);
    m_turningMotorEncoder.setPosition(getAngle().getRadians());
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  /*
   * Configuration values for the module hardware. Specifically the motors and
   * encoders.
   */
  private void configureDevices() {
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
    m_driveMotor.setInverted(module_constants.driveMotorReversed);

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
    m_turningMotorEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);

    /**
     * CTRE Mag Encoder connected to the SparkMAX Absolute/Analog/PWM Duty Cycle
     * input
     * Native will ready 0.0 -> 1.0 for each revolution.
     */
    m_angleEncoder.setZeroOffset(module_constants.angleEncoderOffset);
    m_angleEncoder.setInverted(module_constants.angleEncoderReversed);
    m_angleEncoder.setAverageDepth(8);
    m_angleEncoder.setPositionConversionFactor(module_constants.angleEncoderConversionFactor);
    
    /**
     * Make PID continuous around the 180degree point of the rotation
     */
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.setTolerance(0.001);
    m_simpleTurningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_drivePIDController.setTolerance(0.1, 0.1);
  }
}
