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

    // CAN Motors
    public static final int WHEEL_MOTOR = 0;
    public static final int DRIVE_LEFT_VICTORSPX0 = 1;
    public static final int DRIVE_LEFT_VICTORSPX1 = 2;
    public static final int DRIVE_RIGHT_VICTORSPX0 = 4;
    public static final int DRIVE_RIGHT_VICTORSPX1 = 3;

    // PWM Motors
    public static final int INDEX_MOTOR0 = 3;
    public static final int INTAKE_MOTOR = 4;
    public static final int SHOOTER_LEFT0 = 6;
    public static final int SHOOTER_RIGHT0 = 7;

    // Encoder ports
    public static final int ENCODER_LEFT0 = 1;
    public static final int ENCODER_LEFT1 = 2;
    public static final int ENCODER_RIGHT0 = 5;
    public static final int ENCODER_RIGHT1 = 6;

    // Conversion values
    public static final double INCHES_TO_METERS = 0.0254;
    public static final double ENCODER_TO_INCHES = 2048 / (6 * Math.PI);

    // Solenoids
    public static final int INTAKE_SOLENOID_DEPLOY = 2;
    public static final int INTAKE_SOLENOID_RETRACT = 3;
    public static final int SHOOTER_SOLENOID_DEPLOY = 0;
    public static final int SHOOTER_SOLENOID_RETRACT = 1;

    // Controllers
    public static final int kController0 = 0;
    public static final int kController1 = 1;
}
