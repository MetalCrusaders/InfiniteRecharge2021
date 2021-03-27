// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final Talon m_intakeMotor;
  private final DoubleSolenoid m_intakePistons;
  private boolean intakeIsIn = true;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor = new Talon(Constants.INTAKE_MOTOR);
    m_intakePistons = new DoubleSolenoid(
      Constants.INTAKE_SOLENOID_DEPLOY, 
      Constants.INTAKE_SOLENOID_RETRACT);
  }

  /** Changes the current position of the piston */
  public void pistonPush() {
    if(intakeIsIn) {
      m_intakePistons.set(Value.kForward);
      intakeIsIn = false;
    }
    else {
      m_intakePistons.set(Value.kReverse);
      intakeIsIn = true;
    }
  }

  /** 
   * Sets the intake motor to trigger speed
   * @param input in range [0.0, 1.0] 
   */
  public void intake(double input) {
    if(input > 0.2) {
      m_intakeMotor.set(-input * 0.75); // Reverses direction of trigger input
    }
    else {
      stopIntake();
    }
  }

  /** Stops the intake motor */
  public void stopIntake() {
    m_intakeMotor.set(0);
  }

  /** Puts info in the SmartDashboard */
  public void log() {
    SmartDashboard.putBoolean("Intake is in", intakeIsIn);
    SmartDashboard.putNumber("Intake motor", m_intakeMotor.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }
}
