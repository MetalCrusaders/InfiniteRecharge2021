// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveSequence extends SequentialCommandGroup {

  private final DriveTrain m_driveTrain;
  /** Creates a new DriveSequence. */
  public DriveSequence(DriveTrain driveTrain) {

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

      // // Drives forward 3.15 meters
      new AutoDrive(m_driveTrain, 3.15),

      // Rotates 90 degrees cw
      new CWArc(m_driveTrain, 0.8, 0.35, 50),

      // Rotates 360 degrees CCW
      new CCWArc(m_driveTrain, 0.35, 0.8, 230)
      // new CCWArc(m_driveTrain, 0.35, 0.7, 155)
    );
  }
}
