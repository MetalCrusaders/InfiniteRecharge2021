// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {

  private final VictorSP m_indexMotor;

  /** Creates a new Indexer. */
  public IndexSubsystem() {
    m_indexMotor = new VictorSP(Constants.INDEX_MOTOR0);
  }

  /** 
   * Sets the index motor to joystick speed
   * @param input in range [-1.0, 1.0]
   */
  public void set(double input) {
    m_indexMotor.set(0.85 * input); // 0.7 works for high speed
  }

  /** Stops the index motor */
  public void stop() {
    m_indexMotor.stopMotor();
  }

  /** Puts info in the SmartDashboard */
  public void log() {
    SmartDashboard.putNumber("Index motor", m_indexMotor.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }
}
