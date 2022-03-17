// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private ShooterSubsystem shooterSubsystem;
  /** Creates a new ShooterCommand. */
  public ShooterCommand(DriveSubsystem m_driveSubsystem, ShooterSubsystem m_shooterSubsystem) {
    driveSubsystem = m_driveSubsystem;
    shooterSubsystem = m_shooterSubsystem;
    addRequirements(driveSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setBrakeMode(true);
    shooterSubsystem.turnLimelightOn();
    shooterSubsystem.spoolShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.auxController.getRightTriggerAxis() > 0.75) {
      if (shooterSubsystem.amountOffTarget() > Constants.shooterDeadband) {
        driveSubsystem.centerMotion(true);
      } else if (shooterSubsystem.amountOffTarget() < -Constants.shooterDeadband) {
        driveSubsystem.centerMotion(false);
      } else {
        driveSubsystem.stop();
        shooterSubsystem.shoot(true);
      }
    } else if (RobotContainer.auxController.getLeftTriggerAxis() > 0.75) {
      shooterSubsystem.shoot(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    shooterSubsystem.turnLimelightOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
