// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final DoubleSupplier m_leftSpeed;
  private final DoubleSupplier m_rightSpeed;

  /** Creates a new TankDrive. */
  public TankDrive(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
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
    m_driveTrain.tankDrive(m_leftSpeed.getAsDouble(), m_rightSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
