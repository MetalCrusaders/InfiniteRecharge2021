// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;

public class IndexCommand extends CommandBase {

  private final IndexSubsystem m_indexer;
  private final DoubleSupplier m_input;

  /** Creates a new Index. */
  public IndexCommand(DoubleSupplier input, IndexSubsystem indexer) {
    m_indexer = indexer;
    m_input = input;
    addRequirements(m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexer.set(m_input.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
