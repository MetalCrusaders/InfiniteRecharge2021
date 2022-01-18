// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  VictorSP m_leftShooter;
  VictorSP m_rightShooter;
  private final DoubleSolenoid m_shooterPistons;

  private final double shooter_threshold = 0.2;
  private boolean shooterIsIn = true;
  private final double limit = 0.75;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_leftShooter = new VictorSP(Constants.SHOOTER_LEFT0);
    m_rightShooter = new VictorSP(Constants.SHOOTER_RIGHT0);
    m_shooterPistons = new DoubleSolenoid(
      Constants.SHOOTER_SOLENOID_DEPLOY, 
      Constants.SHOOTER_SOLENOID_RETRACT);
  }

  public void shooterPush() {
    if(shooterIsIn) {
      m_shooterPistons.set(Value.kForward);
      shooterIsIn = false;
    }
    else {
      m_shooterPistons.set(Value.kReverse);
      shooterIsIn = true;
    }
  }

  /**
   * Sets the shooter motors to input speed
   * 
   * @param input in range [-1.0, 1.0]
   */
  public void shoot(double input) {
    if (input > shooter_threshold) {
      m_leftShooter.set(-input * limit);
      m_rightShooter.set(input * limit);
    }
    else {
      stop();
    }
  }

  /** Stops the shooter motors */
  public void stop() {
    m_leftShooter.set(0);
    m_rightShooter.set(0);
  }

  public void log() {
    SmartDashboard.putNumber("Left shooter", m_leftShooter.get());
    SmartDashboard.putNumber("Right shooter", m_rightShooter.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }
}
