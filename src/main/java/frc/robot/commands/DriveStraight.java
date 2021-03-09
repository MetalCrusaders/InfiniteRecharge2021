// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveStraight extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final DoubleSupplier m_speed;

  private static final double angleSetpoint = 0;
  private static final double MAX_STRAIGHT_SPEED = 0.75;
  private static final double kP = 0.001;

  /** Creates a new DriveStraight. */
  public DriveStraight(DoubleSupplier speed, DriveTrain driveTrain) {
    m_speed = speed;
    m_driveTrain = driveTrain;

    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_driveTrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_speed.getAsDouble() * MAX_STRAIGHT_SPEED;
    double turnValue = kP * (angleSetpoint - m_driveTrain.getHeading());
    turnValue = Math.copySign(turnValue, speed);
    m_driveTrain.arcadeDrive(speed, turnValue);
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
