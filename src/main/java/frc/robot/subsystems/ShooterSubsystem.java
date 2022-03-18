// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax flywheel = new CANSparkMax(Constants.motorFlyWheel, MotorType.kBrushless);
  private TalonSRX feed = new TalonSRX(Constants.motorFeed);
  private SparkMaxPIDController PID = flywheel.getPIDController();

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry ledMode = table.getEntry("ledMode");
  private NetworkTableEntry camMode = table.getEntry("camMode");
  private double x, y, area, testHRPM, testLRPM/* , calcHighRPM */;

  public void shooterInit() {
    flywheel.setInverted(false);
    feed.setInverted(false);

    turnLimelightOff();
    PIDSetup();
  }

  public void PIDSetup() {
    PID.setP(Constants.flyP);
    PID.setI(Constants.flyI);
    PID.setD(Constants.flyD);
    PID.setFF(Constants.flyFF);
  }

  public void turnLimelightOff() {
    ledMode.setDouble(3);
    camMode.setDouble(0);
  }

  @Override
  public void periodic() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    testHRPM = SmartDashboard.getNumber("High RPM", Constants.highRPM);
    testLRPM = SmartDashboard.getNumber("Low RPM", Constants.lowRPM);
  }

  public void turnLimelightOn() {
    ledMode.setDouble(1);
    camMode.setDouble(1);
  }

  public double amountOffTarget() {
    return x;
  }

  public void shoot(boolean high) {
    if (high) {
      PID.setReference(testHRPM, ControlType.kVelocity);
      // PID.setReference(calcHighRPM(), ControlType.kVelocity);
      feed.set(ControlMode.PercentOutput, Constants.feedSpeed);
    } else {
      PID.setReference(testLRPM, ControlType.kVelocity);
      // PID.setReference(Constants.lowRPM, ControlType.kVelocity);
    }
  }

  public double calcHighRPM() {
    double distance, targetAngleRad, rpm;

    targetAngleRad = (Constants.cameraAngleDeg + y) * (Math.PI / 180);

    distance = (Constants.targetHeightInch - Constants.cameraHeightInch) / (Math.tan(targetAngleRad));

    rpm = distance;

    return rpm;

  }

  public void spoolShooter() {
    PID.setReference(testLRPM, ControlType.kVelocity);
    // PID.setReference(Constants.lowRPM, ControlType.kVelocity);
    // PID.setReference(Constants.spoolRPM, ControlType.kVelocity);
  }

  public void stop() {
    PID.setReference(0, ControlType.kVelocity);
  }
}