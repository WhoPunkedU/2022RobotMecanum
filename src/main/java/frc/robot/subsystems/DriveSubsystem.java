// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private WPI_TalonFX motorFL = new WPI_TalonFX(Constants.motorFL),
                      motorBL = new WPI_TalonFX(Constants.motorBL),
                      motorFR = new WPI_TalonFX(Constants.motorFR),
                      motorBR = new WPI_TalonFX(Constants.motorBR);

  private MotorControllerGroup leftMotors = new MotorControllerGroup(motorFL, motorBL),
                                rightMotors = new MotorControllerGroup(motorFR, motorBR);
  
  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
  
  private AHRS gyro = new AHRS(Port.kMXP);
  private boolean setupCompleted = false;
  private int a = Constants.driveScalingCoefficient;

  public void driveInit() {

    if (!setupCompleted) {

      motorFL.setInverted(TalonFXInvertType.Clockwise);
      motorBL.setInverted(TalonFXInvertType.CounterClockwise);
      motorFR.setInverted(TalonFXInvertType.Clockwise);
      motorBR.setInverted(TalonFXInvertType.CounterClockwise);

      motorFL.setSelectedSensorPosition(0);
      motorBL.setSelectedSensorPosition(0);
      motorFR.setSelectedSensorPosition(0);
      motorBR.setSelectedSensorPosition(0);

      motorFL.configVoltageCompSaturation(Constants.maxVoltage);
      motorBL.configVoltageCompSaturation(Constants.maxVoltage);
      motorFR.configVoltageCompSaturation(Constants.maxVoltage);
      motorBR.configVoltageCompSaturation(Constants.maxVoltage);

      motorFL.enableVoltageCompensation(Constants.voltageCompenstaion);
      motorBL.enableVoltageCompensation(Constants.voltageCompenstaion);
      motorFR.enableVoltageCompensation(Constants.voltageCompenstaion);
      motorBR.enableVoltageCompensation(Constants.voltageCompenstaion);

      gyro.reset();

      setupCompleted = true;

    }
  }

  public void mecanumDriveJoystick(double x, double y, double z) {
    double FL, FR, BL, BR, optimize;

    x = (a*(Math.pow(x, 3)) + (1-a)*x);
    y = -(a*(Math.pow(y, 3)) + (1-a)*y);
    z = -(a*(Math.pow(z, 3)) + (1-a)*z);

    FL = x+y+z;
    BL = x-y+z;
    FR = x-y-z;
    BR = x+y-z;

    optimize = Math.max(Math.max(Math.max(Math.max(FL, BL), FR), BR), 1.00);

    FL = (FL/optimize)*Constants.maxDriveSpeed;
    BL = (BL/optimize)*Constants.maxDriveSpeed;
    FR = (FR/optimize)*Constants.maxDriveSpeed;
    BR = (BR/optimize)*Constants.maxDriveSpeed;

    mecanumDrive(FL, BL, FR, BR);
  }

  public void mecanumDrive(double FL, double BL, double FR, double BR) {
    motorFL.set(ControlMode.PercentOutput, FL);
    motorBL.set(ControlMode.PercentOutput, BL);
    motorFR.set(ControlMode.PercentOutput, FR);
    motorBR.set(ControlMode.PercentOutput, BR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
