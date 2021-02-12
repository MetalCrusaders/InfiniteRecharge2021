// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {

  private final VictorSP m_leftIndexer;
  private final VictorSP m_rightIndexer;

  /** Creates a new Indexer. */
  public IndexSubsystem() {
    m_leftIndexer = new VictorSP(Constants.INDEX_MOTOR0);
    m_rightIndexer = new VictorSP(Constants.INDEX_MOTOR1);
  }

  /** 
   * Sets the index motor to joystick speed
   * @param input in range [-1.0, 1.0]
   */
  public void set(double input) {
    if(input > 0.2) {
      m_leftIndexer.set(-0.5 * input);
      m_rightIndexer.set(0.5 * input);
    }
    else {
      stop();
    }
  }

  /** Stops the index motor */
  public void stop() {
    m_leftIndexer.stopMotor();
    m_rightIndexer.stopMotor();
  }

  /** Puts info in the SmartDashboard */
  public void log() {
    SmartDashboard.putNumber("Left motor", m_leftIndexer.get());
    SmartDashboard.putNumber("Right motor", m_rightIndexer.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }
}
