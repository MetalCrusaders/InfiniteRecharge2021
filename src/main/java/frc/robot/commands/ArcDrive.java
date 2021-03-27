// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ArcDrive extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final double DISTANCE_BETWEEN_WHEELS = 21.0; // inches
  private double radius; // In inches
  private double angleSweep; // Positive angle -> Clockwise
  private double baseVel; 
  private double distanceSweep; 

  private double leftRad;
  private double rightRad;

  private double leftVel;
  private double rightVel;

  private double leftIn;
  private double rightIn;

  private double desiredRatio; // Left to right
  private double currentRatio;
  private double ratioError;
  private double k_ratio = 1.0;
  private double kBalance;

  private double MAX_SPEED = 1.0;

  /** Creates a new ArcDrive. */
  public ArcDrive(DriveTrain driveTrain, double inRadius, double inAngleSweep, double inBaseVel) {
    
    m_driveTrain = driveTrain;
    radius = inRadius;
    angleSweep = inAngleSweep;
    baseVel = inBaseVel;
    distanceSweep = 2 * Math.PI * radius * (angleSweep / 360);

    leftRad = radius - DISTANCE_BETWEEN_WHEELS / 2;
    rightRad = radius + DISTANCE_BETWEEN_WHEELS / 2;

    desiredRatio = leftRad / rightRad;

    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoders();
    m_driveTrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ratioError = getRatioError();

    kBalance = 1 + k_ratio * ratioError;  // Is < 1 for left slower than right, > 1 for left faster than right
    m_driveTrain.tankDrive(-baseVel * desiredRatio, -baseVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_driveTrain.getAverageEncoderDistance() + distanceSweep) < 1) {
      return true;
    }
    else {
      return false;
    }
  }

  public double getRatioError() {
    leftVel = m_driveTrain.getLeftEncoderRate();
    rightVel = m_driveTrain.getRightEncoderRate();
    currentRatio = leftVel / rightVel;

    return currentRatio - desiredRatio;
  }

}
