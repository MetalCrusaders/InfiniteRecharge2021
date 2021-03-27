// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.BangDrive;
import frc.robot.commands.CCWArc;
import frc.robot.commands.CWArc;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSlalom extends SequentialCommandGroup {

  private final DriveTrain m_driveTrain;
  /** Creates a new AutoSlalom. */
  public AutoSlalom(DriveTrain driveTrain) {

    m_driveTrain = driveTrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Drives forward 54 cm
      new BangDrive(m_driveTrain, 0.2, 0.5),

      // Rotates 90 degrees CCW
      new CCWArc(m_driveTrain, 0.35, 0.7, 50),
      
      // Rotates 90 degrees CW
      new CWArc(m_driveTrain, 0.8, 0.35, 9), // 11 goes straight

      // // Drives forward 3.20 meters (3.17 hits D8)
      new AutoDrive(m_driveTrain, 3.20),

      // Rotates 90 degrees cw
      new CWArc(m_driveTrain, 0.83, 0.35, 40),

      // Rotates 360 degrees CCW
      new CCWArc(m_driveTrain, 0.3, 0.8, 125),
      new CCWArc(m_driveTrain, 0.5, 0.8, 135),

      // Rotates 70 degrees CW
      new CWArc(m_driveTrain, 0.8, 0.4, 6),

      // Drives forward 3.04 meters
      new BangDrive(m_driveTrain, 2.70, 0.75),

      // Rotates 30 degrees CW (hits D2)
      // new CWArc(m_driveTrain, 0.8, 0.3, 10)

      // Rotates 90 degrees CW
      new CWArc(m_driveTrain, 0.85, 0.3, 30),

      // Rotates 90 degrees CCW
      new CCWArc(m_driveTrain, 0.3, 0.85, 20)
    );
  }
}
