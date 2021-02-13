// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class RotateDrive extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final double m_angle;

  private static final double kP = 1;

  /** Creates a new RotateDrive. */
  public RotateDrive(double angle, DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    m_angle = angle;

    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnValue = (m_angle - m_driveTrain.getHeading()) * kP;
    m_driveTrain.arcadeDrive(0, turnValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
