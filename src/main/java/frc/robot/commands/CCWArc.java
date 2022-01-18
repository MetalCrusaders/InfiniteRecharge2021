// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class CCWArc extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double m_leftVel;
  private double m_rightVel;
  private double m_angle;

  /** Creates a new CCWArc. */
  public CCWArc(DriveTrain driveTrain, double leftVel, double rightVel, double angle) {
    m_driveTrain = driveTrain;
    m_leftVel = leftVel;
    m_rightVel = rightVel;
    m_angle = angle;

    // Use addRequirements() here to declare subsystem dependencies.
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
    // double leftVel = m_driveTrain.getLeftEncoderRate();
    // double rightVel = m_driveTrain.getRightEncoderRate();
    // currentRatio = rightVel / leftVel;

    // ratioError = idealRatio - currentRatio;

    // kBalance = 1 + kRatio * ratioError;

    m_driveTrain.tankDrive(m_leftVel, m_rightVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_driveTrain.getHeading() < -m_angle) {
      return true;
    }
    else {
      return false;
    }
  }
}
