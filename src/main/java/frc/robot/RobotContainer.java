// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final SwerveSubsystem swerveDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  SwerveInputStream driveFieldAngularVelocityStream;
  SwerveInputStream driveRobotAngularVelocityStream;

  int DebugMode = 0;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverJoystick =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureInputStreams();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    swerveDrive.setDefaultCommand(swerveDrive.driveFieldOrientedCommand(driveRobotAngularVelocityStream));

  }

  private void configureInputStreams() {
    driveFieldAngularVelocityStream = SwerveInputStream.of(
      swerveDrive.swerveDrive,
      () -> driverJoystick.getLeftY(),
      () -> (DebugMode == 0 ? driverJoystick.getLeftX() : 0.0)
    ).withControllerRotationAxis(() -> DebugMode == 0 ? driverJoystick.getRightX() : 0.0)
     .deadband(Constants.OperatorConstants.deadband)
     .allianceRelativeControl(true);

    driveRobotAngularVelocityStream = driveFieldAngularVelocityStream.copy()
    .robotRelative(true)
    .allianceRelativeControl(false);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return swerveDrive.changePosition(new Translation2d(0.0, 2.0), Units.feetToMeters(3.0));
  }
}
