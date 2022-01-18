// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class RotateDrive extends CommandBase {

  private final DriveTrain m_driveTrain;
  private double kP = 0.02;
  private double kTurn;
  private double angle;
  private double error;

  private double MAX_ROTATE = 1;


  /** Creates a new RotateDrive. */
  public RotateDrive(DriveTrain driveTrain, double inAngle) {
    m_driveTrain = driveTrain;
    angle = inAngle;


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

    error = Math.abs(m_driveTrain.getHeading() - angle);
    kTurn = kP * error;
    if(kTurn > MAX_ROTATE) {
      kTurn = MAX_ROTATE;
    }
    kTurn *= Math.signum(angle);
    m_driveTrain.tankDrive(-kTurn, kTurn);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(error) < 3) {
      return true;
    }
    return false;
  }
}
