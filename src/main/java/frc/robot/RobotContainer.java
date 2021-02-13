// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final IndexSubsystem m_indexer = new IndexSubsystem();

  private final XboxController m_controller = new XboxController(Constants.kController0);

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final AutoDrive m_autoDrive = new AutoDrive(m_driveTrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default commands
    setDefaultCommands();

    // Show what command each subsystem is running on the SmartDashboard
    SmartDashboard.putData(m_driveTrain);
  }

  /** Set default commands for subsystems based on controller input*/
  private void setDefaultCommands() {
    // m_driveTrain.setDefaultCommand(
    //   new ArcadeDrive(
    //     () -> m_controller.getY(Hand.kLeft), () -> m_controller.getX(Hand.kLeft), m_driveTrain)
    // );

    m_driveTrain.setDefaultCommand(
      new TankDrive(
        () -> m_controller.getY(Hand.kLeft), () -> m_controller.getY(Hand.kRight), m_driveTrain)
    );

    m_indexer.setDefaultCommand(
      new IndexCommand(
        () -> m_controller.getTriggerAxis(Hand.kLeft), m_indexer)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoDrive;
  }
}
