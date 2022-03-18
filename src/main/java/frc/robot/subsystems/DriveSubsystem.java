// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private WPI_TalonFX motorFL = new WPI_TalonFX(Constants.motorFL),
      motorBL = new WPI_TalonFX(Constants.motorBL),
      motorFR = new WPI_TalonFX(Constants.motorFR),
      motorBR = new WPI_TalonFX(Constants.motorBR);

  // private MotorControllerGroup leftMotors = new MotorControllerGroup(motorFL,
  // motorBL),
  // rightMotors = new MotorControllerGroup(motorFR, motorBR);

  // private PIDController leftController = new PIDController(Constants.driveP,
  // Constants.driveI, Constants.driveD),
  // rightController = new PIDController(Constants.driveP, Constants.driveI,
  // Constants.driveD);

  // private DifferentialDrive differentialDrive = new
  // DifferentialDrive(leftMotors, rightMotors);

  private AHRS gyro = new AHRS(Port.kMXP);
  private int a = Constants.driveScalingCoefficient;
  private SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.xRateLimit),
      yLimiter = new SlewRateLimiter(Constants.yRateLimit),
      zLimiter = new SlewRateLimiter(Constants.zRateLimit);

  public void driveInit() {
    motorFL.setInverted(TalonFXInvertType.Clockwise);
    motorBL.setInverted(TalonFXInvertType.CounterClockwise);
    motorFR.setInverted(TalonFXInvertType.Clockwise);
    motorBR.setInverted(TalonFXInvertType.CounterClockwise);

    motorFL.configVoltageCompSaturation(Constants.maxVoltage);
    motorBL.configVoltageCompSaturation(Constants.maxVoltage);
    motorFR.configVoltageCompSaturation(Constants.maxVoltage);
    motorBR.configVoltageCompSaturation(Constants.maxVoltage);

    motorFL.enableVoltageCompensation(Constants.voltageCompenstaion);
    motorBL.enableVoltageCompensation(Constants.voltageCompenstaion);
    motorFR.enableVoltageCompensation(Constants.voltageCompenstaion);
    motorBR.enableVoltageCompensation(Constants.voltageCompenstaion);

    motorFL.setSafetyEnabled(Constants.motorSafteyEnabled);
    motorFR.setSafetyEnabled(Constants.motorSafteyEnabled);
    motorBL.setSafetyEnabled(Constants.motorSafteyEnabled);
    motorBR.setSafetyEnabled(Constants.motorSafteyEnabled);
  }

  public void setBrakeMode(boolean brake) {
    if (brake) {
      motorFL.setNeutralMode(NeutralMode.Brake);
      motorBL.setNeutralMode(NeutralMode.Brake);
      motorFR.setNeutralMode(NeutralMode.Brake);
      motorBR.setNeutralMode(NeutralMode.Brake);
    } else {
      motorFL.setNeutralMode(NeutralMode.Coast);
      motorBL.setNeutralMode(NeutralMode.Coast);
      motorFR.setNeutralMode(NeutralMode.Coast);
      motorBR.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void zeroDriveEncoders() {
    motorFL.setSelectedSensorPosition(0);
    motorBL.setSelectedSensorPosition(0);
    motorFR.setSelectedSensorPosition(0);
    motorBR.setSelectedSensorPosition(0);
  }

  public void zeroGyroYaw() {
    gyro.reset();
  }

  public void mecanumDriveJoystick(double x, double y, double z) {
    double FL, FR, BL, BR, optimize;

    x = xLimiter.calculate(x);
    y = yLimiter.calculate(y);
    z = zLimiter.calculate(z);

    x = (a * (Math.pow(x, 3)) + (1 - a) * x);
    y = -(a * (Math.pow(y, 3)) + (1 - a) * y);
    z = -(a * (Math.pow(z, 3)) + (1 - a) * z);

    FL = x + y + z;
    BL = x - y + z;
    FR = x - y - z;
    BR = x + y - z;

    optimize = Math.max(Math.max(Math.max(Math.max(FL, BL), FR), BR), 1.00);

    FL = (FL / optimize) * Constants.maxDriveSpeed;
    BL = (BL / optimize) * Constants.maxDriveSpeed;
    FR = (FR / optimize) * Constants.maxDriveSpeed;
    BR = (BR / optimize) * Constants.maxDriveSpeed;

    mecanumDrive(FL, BL, FR, BR);
  }

  public void centerMotion(boolean right) {
    if (right) {
      mecanumDrive(Constants.targetSpeed, Constants.targetSpeed, -Constants.targetSpeed, -Constants.targetSpeed);
    } else {
      mecanumDrive(-Constants.targetSpeed, -Constants.targetSpeed, Constants.targetSpeed, Constants.targetSpeed);
    }
  }

  public void stop() {
    mecanumDrive(0, 0, 0, 0);
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
