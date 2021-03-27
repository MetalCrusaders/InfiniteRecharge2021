// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootSpeeds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootHigh extends CommandBase {

  private final ShooterSubsystem m_ShooterSubsystem;
  private final double highShootSpeed = 1;  // Speed should remain constant after determining

  /** Creates a new ShootHigh. */
  public ShootHigh(ShooterSubsystem shooterSubsystem) {
    m_ShooterSubsystem = shooterSubsystem;

    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.shoot(highShootSpeed);
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
