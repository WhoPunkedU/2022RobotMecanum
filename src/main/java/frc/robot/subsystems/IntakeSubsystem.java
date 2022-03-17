// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  
  private TalonSRX motor = new TalonSRX(Constants.motorINT);
  private DoubleSolenoid solenoid = new DoubleSolenoid(Constants.pneumaticHub, PneumaticsModuleType.REVPH, Constants.solenoidIntakeForward, Constants.solenoidIntakeReverse);

  public void intakeInit() {
    motor.setInverted(false);
    motor.configVoltageCompSaturation(Constants.maxVoltage);
    motor.enableVoltageCompensation(Constants.voltageCompenstaion);
    solenoid.set(Value.kReverse);
  }

  public void Intake(boolean input) {
    if (input) {
      motor.set(ControlMode.PercentOutput, Constants.intakeSpeed);
      solenoid.set(Value.kForward);
    } else {
      solenoid.set(Value.kReverse);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}