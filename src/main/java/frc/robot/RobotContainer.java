// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  // An autonomous routine that shoots the loaded frisbees
  Command m_autonomousCommand = new InstantCommand();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created via the button
   * factories on {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID} or one of its
   * subclasses ({@link edu.wpi.first.wpilibj2.command.button.CommandJoystick} or {@link
   * CommandXboxController}).
   */
  private void configureButtonBindings() {
    // Configure your button bindings here

    // We can bind commands while retaining references to them in RobotContainer

    // Spin up the shooter when the 'A' button is pressed
    // m_driverController.a().onTrue(m_spinUpShooter);

    // Turn off the shooter when the 'B' button is pressed
    // m_driverController.b().onTrue(m_stopShooter);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }
}