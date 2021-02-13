// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final DoubleSupplier m_speed;
  private final DoubleSupplier m_rotate;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(DoubleSupplier speed, DoubleSupplier rotate, DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    m_speed = speed;
    m_rotate = rotate;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.arcadeDrive(m_speed.getAsDouble(), m_rotate.getAsDouble());
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
