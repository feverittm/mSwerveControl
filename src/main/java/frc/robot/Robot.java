// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.NavXSwerve;
import frc.robot.utils.SwerveModuleConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public SwerveModule module;
  // The gyro sensor
  public static NavXSwerve m_gyro;
  double joy_angle = 0.0;
  double joy_drive = 0;
  int angle_count = 0;

  public Joystick m_driverController = new Joystick(0);

  SwerveModuleConstants config;

  @Override
  public void robotInit() {
    m_gyro = new NavXSwerve(SPI.Port.kMXP);
    config = ModuleConstants.Front_Left_Configuration;
    module = new SwerveModule(ModuleConstants.Front_Left_Configuration);
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    module.setDesiredState(new SwerveModuleState(0.0, new Rotation2d((Math.PI)/(2))));
  }

  @Override
  public void teleopInit() {
    SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(0.0));
    module.setDesiredState(state);
  }

  @Override
  public void teleopPeriodic() {
    joy_angle = -m_driverController.getRawAxis(0) * (2*Math.PI);
    joy_drive = -m_driverController.getRawAxis(2);
    SmartDashboard.putNumber("Joystick Angle", joy_angle);
    SwerveModuleState state = new SwerveModuleState(joy_drive, new Rotation2d(joy_angle));
    module.setDesiredState(state);
  }

  @Override
  public void robotPeriodic() {
    // swerve module state
    SwerveModuleState state = module.getState();
    SmartDashboard.putNumber("Module Number", config.moduleNumber);
    SmartDashboard.putNumber("Module State - Velocity: ", state.speedMetersPerSecond);
    SmartDashboard.putNumber("Module State - Angle: ", state.angle.getDegrees());
    SmartDashboard.putNumber("Module Position - Distance: ", module.getPosition().distanceMeters);
    SmartDashboard.putNumber("Module Position - Angle: ", module.getPosition().angle.getDegrees());
    // raw hardware
    SmartDashboard.putNumber("Raw Turning Motor Angle", module.getRawAngle());
    SmartDashboard.putNumber("Module Angle", module.getAngle().getDegrees());
    SmartDashboard.putBoolean("Module Zeroed", (module.getAngle().getRadians() == 0.0));
    SmartDashboard.putNumber("Absolute Encoder Offset", module.getAbsoluteOffset());
    SmartDashboard.putBoolean("Absolute Encoder Inversion", module.getAbsoluteEncoderInversion());

    //
    SmartDashboard.putNumber("Gyro YAW", m_gyro.getYaw());
    SmartDashboard.putNumber("Gyro Rotation", m_gyro.getRotation3d().getAngle());
    SmartDashboard.putBoolean("Gyro Connected", m_gyro.isConnected());
  }
}
