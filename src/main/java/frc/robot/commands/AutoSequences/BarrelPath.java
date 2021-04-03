// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BangDrive;
import frc.robot.commands.CCWArc;
import frc.robot.commands.CWArc;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BarrelPath extends SequentialCommandGroup {

  private final DriveTrain m_driveTrain;
  /** Creates a new DriveSequence. */
  public BarrelPath(DriveTrain driveTrain) {

    m_driveTrain = driveTrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //move forward 90 cm  (1.8, 0.71)
      new BangDrive(m_driveTrain, 1.83, 0.71), //0.9, 0.8 too short, 0.7, 0.6 speed for the new battery

      //first half of D5 arc
      //angles tested: (0.7, 0.22, 250) ,(0.7, 0.22, 220) 
      new CWArc(m_driveTrain, 0.8, 0.27, 220),

      //move forward 
      new BangDrive(m_driveTrain, 0.4, 0.8),
      
      //second half of D5 arc 
      //angles tested: 0.7, 0.5, 50, 90 degrees
      new CWArc(m_driveTrain, 0.8, 0.5, 97),

      //travels until B8
      //B -> D = 1.0m -> 0.7m 
      new BangDrive(m_driveTrain, 0.6, 0.8),

      //first half of B8 arc (0.3, 0.9, 150)
      new CCWArc(m_driveTrain, 0.27, 0.6428, 190),

      // drive straight of B8 
      new BangDrive(m_driveTrain, 0.5, 0.8),

      //second half of B8 arc
      new CCWArc(m_driveTrain, 0.35, 0.75, 60),
      
      // B8 to D10
      new BangDrive(m_driveTrain, 1.7, 0.8),
      
      //first half of D10 arc .35, .9
      new CCWArc(m_driveTrain, 0.31, 0.86, 148),

      //drives back 220 cm
      new BangDrive(m_driveTrain, 5, 1)
     );
  }
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  // april fools !!!!
}
