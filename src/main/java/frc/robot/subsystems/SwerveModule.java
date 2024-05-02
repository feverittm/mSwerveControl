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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ModuleConstants.SwervePID;
import frc.robot.utils.SwerveModuleConstants;

public class SwerveModule {
  private SwerveModuleConstants module_constants;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveMotorEncoder;
  private final RelativeEncoder m_turningMotorEncoder;
  private final SparkAbsoluteEncoder m_angleEncoder;

  private PIDController m_TurningPIDController = new PIDController(SwervePID.p, SwervePID.i, SwervePID.d);
  private PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  private final double DRIVE_REDUCTION = 1.0 / 6.75;
  private final double NEO_FREE_SPEED = 5820.0 / 60.0;
  private final double WHEEL_DIAMETER = 0.1016;
  public static final double dist = Units.inchesToMeters(11.5);
  private final double MAX_VELOCITY = NEO_FREE_SPEED * DRIVE_REDUCTION * WHEEL_DIAMETER * Math.PI;
  private final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (dist / Math.sqrt(2.0));
  private final double MAX_VOLTAGE = 12;

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
    m_angleEncoder.setPositionConversionFactor(1.0);
    m_angleEncoder.setVelocityConversionFactor(1.0);

    double driveReduction = 1.0 / 6.75;
    double WHEEL_DIAMETER = 0.1016;
    double rotationsToDistance = driveReduction * WHEEL_DIAMETER * Math.PI;

    m_turningMotorEncoder.setPosition(0);
    m_driveMotorEncoder.setPosition(0);
    m_driveMotorEncoder.setPositionConversionFactor(rotationsToDistance);
    m_driveMotorEncoder.setVelocityConversionFactor(rotationsToDistance / 60);

    m_TurningPIDController.enableContinuousInput(-180, 180);
    m_driveMotor.setInverted(module_constants.driveMotorReversed);
    m_turningMotor.setInverted(module_constants.angleMotorReversed);

    m_turningMotor.setSmartCurrentLimit(ModuleConstants.ANGLE_CURRENT_LIMIT);
    m_driveMotor.setSmartCurrentLimit(ModuleConstants.DRIVE_CURRENT_LIMIT);
  }

  /**
   * Get the raw value from the absolute encoder on the SparkMax
   * 
   * @return raw angle (0.0->360.0)
   */
  public double getRawAngle() {
    return m_angleEncoder.getPosition();
  }

  public double getAngle() {
    return getRawAngle() * 360.0;
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
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getAngle());
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
    return new SwerveModuleState(getDriveEncoderVelocity(), getRotation());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveEncoderPosition(), getRotation());
  }

  private void drive(double speedMetersPerSecond, double angle) {
    double voltage = (speedMetersPerSecond / MAX_VELOCITY) * MAX_VOLTAGE;
    m_driveMotor.setVoltage(voltage);
    m_turningMotor.setVoltage(-m_TurningPIDController.calculate(this.getAngle(), angle));
  }

  public void drive(SwerveModuleState state) {
    SwerveModuleState optimized = SwerveModuleState.optimize(state, new Rotation2d(Math.toRadians(getAngle())));
    this.drive(optimized.speedMetersPerSecond, optimized.angle.getDegrees());
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  /**
   * Sets the desired state for the module.
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // SwerveModuleState state = SwerveModuleState.optimize(desiredState,
    // getState().angle);
    SwerveModuleState target = SwerveModuleState.optimize(desiredState, getState().angle);
    SwerveModuleState state = desiredState;

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(getDriveEncoderVelocity(), state.speedMetersPerSecond);
    SmartDashboard.putNumber("State/Mps", state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_TurningPIDController.calculate(m_angleEncoder.getPosition(), state.angle.getRadians());

    SmartDashboard.putNumber("State/Setpoint", m_TurningPIDController.getSetpoint());
    SmartDashboard.putNumber("State/driveOutput", driveOutput);
    SmartDashboard.putNumber("State/turnOutput", turnOutput);
    SmartDashboard.putNumber("Optimized Angle", target.angle.getDegrees());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
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
    // m_angleEncoder.setPositionConversionFactor(2*Math.PI);
    m_angleEncoder.setAverageDepth(8);
    m_angleEncoder.setPositionConversionFactor(module_constants.angleEncoderConversionFactor);

    /**
     * Make PID continuous around the 180degree point of the rotation
     * 0
     * PI/2 + -PI/2
     * PI/-PI
     * Note that + angle goes CCW from the zero point of the encoder.
     */
    m_TurningPIDController = new PIDController(
        ModuleConstants.kPModuleTurningController,
        0,
        0);

    m_TurningPIDController.enableContinuousInput(0, 2 * Math.PI);
    m_TurningPIDController.setTolerance(0.001);
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("module/EncoderZero", m_angleEncoder.getZeroOffset());
    SmartDashboard.putBoolean("module/AngleInverted", m_angleEncoder.getInverted());
    SmartDashboard.putBoolean("module/DriveInverted", m_driveMotor.getInverted());
    SmartDashboard.putBoolean("module/TurnInverted", m_turningMotor.getInverted());
    SmartDashboard.putNumber("module/EncoderCF", m_angleEncoder.getPositionConversionFactor());
    SmartDashboard.putNumber("module/DriveCF", m_driveMotorEncoder.getPositionConversionFactor());
    SmartDashboard.putNumber("module/AnglePos", m_turningMotorEncoder.getPosition());
  }
}
