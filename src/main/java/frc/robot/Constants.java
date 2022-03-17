// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int rightBumper = 6;

    public static final int motorFL = 2;
    public static final int motorBL = 3;
    public static final int motorFR = 0;
    public static final int motorBR = 1;
    public static final int motorFlyWheel = 7;
    public static final int motorFeed = 5;
    public static final int motorINT = 4;
    public static final int pneumaticHub = 8;

    public static final int solenoidIntakeForward = 2;
    public static final int solenoidIntakeReverse = 1;

    public static final double maxDriveSpeed = 1;
    public static final double intakeSpeed = 0.5;
    public static final double feedSpeed = 0;
    public static final double targetSpeed = 0.3;

    public static final double flyP = 0;
    public static final double flyI = 0;
    public static final double flyD = 0;
    public static final double flyFF = 0;

    public static final double driveP = 0;
    public static final double driveI = 0;
    public static final double driveD = 0;
    public static final double maxVoltage = 11;
    public static final boolean voltageCompenstaion = true;
    public static final boolean motorSafteyEnabled = true;

    public static final double shooterDeadband = 0;
    public static final double highRPM = 0;
    public static final double lowRPM = 0;

    public static final double xRateLimit = 0;
    public static final double yRateLimit = 0;
    public static final double zRateLimit = 0;
    public static final int driveScalingCoefficient = 0;

}
