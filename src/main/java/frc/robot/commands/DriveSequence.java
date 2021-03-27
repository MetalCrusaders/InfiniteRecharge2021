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
      // Drives forward 54 inches (testing for coast)
      new AutoDrive(m_driveTrain, 54),
      
      // Rotates 90 degrees ccw
      new RotateDrive(m_driveTrain, -90),

      // Drives forward 84 inches
      new AutoDrive(m_driveTrain, 84),

      // Rotates 90 degrees cw
      new RotateDrive(m_driveTrain, 90)
    );
  }
}
