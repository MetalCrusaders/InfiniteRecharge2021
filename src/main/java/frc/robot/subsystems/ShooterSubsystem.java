// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  VictorSP m_leftShooter;
  VictorSP m_rightShooter;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // m_leftShooter = new VictorSP(Constants.SHOOTER_LEFT0);
    // m_rightShooter = new VictorSP(Constants.SHOOTER_RIGHT0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
