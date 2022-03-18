// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.autonomous.AutonomousCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static XboxController drivController = new XboxController(0);
  public static XboxController auxController = new XboxController(1);

  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final LiftSubsystem liftSubsystem = new LiftSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
  private final LiftCommand liftCommand = new LiftCommand(liftSubsystem);
  private final ShooterCommand shooterCommand = new ShooterCommand(driveSubsystem, shooterSubsystem);
  private final AutonomousCommand autonomousCommand = new AutonomousCommand();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    CommandScheduler.getInstance().registerSubsystem(driveSubsystem,
        intakeSubsystem,
        liftSubsystem,
        shooterSubsystem);
    driveSubsystem.setDefaultCommand(driveCommand);
    intakeSubsystem.setDefaultCommand(intakeCommand);
    liftSubsystem.setDefaultCommand(liftCommand);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(auxController, Constants.rightBumper).whenHeld(shooterCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonomousCommand;
  }
}
