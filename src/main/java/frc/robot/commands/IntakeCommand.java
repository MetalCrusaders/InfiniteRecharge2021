// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

  private final IntakeSubsystem m_intakeSubsystem;
  private final DoubleSupplier m_input;
  private final BooleanSupplier m_button;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(DoubleSupplier input, BooleanSupplier button, IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_input = input;
    m_button = button;
    
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_button.getAsBoolean()) {
      m_intakeSubsystem.pistonPush();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.intake(m_input.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
