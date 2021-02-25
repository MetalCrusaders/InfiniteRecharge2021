// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakePistons;
// import frc.robot.commands.RotateDrive;
// import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final IndexSubsystem m_indexer = new IndexSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  // private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  private final CrusaderController m_controller = new CrusaderController(Constants.kController0);

  private final AutoDrive m_autoDrive = new AutoDrive(m_driveTrain);
  // private final RotateDrive m_rotateDrive = new RotateDrive(90, m_driveTrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default commands
    setDefaultCommands();

    // Show what command each subsystem is running on the SmartDashboard
    SmartDashboard.putData(m_driveTrain);
    SmartDashboard.putData(m_indexer);
    SmartDashboard.putData(m_intake);
    // SmartDashboard.putData(m_shooter);
  }

  /** Set default commands for subsystems based on controller input*/
  private void setDefaultCommands() {

    m_driveTrain.setDefaultCommand(
      new TankDrive(
        () -> m_controller.getLeftStickY(), () -> m_controller.getRightStickY(), m_driveTrain)
    );

    // m_shooter.setDefaultCommand(
    //   new ShooterCommand(
    //     () -> m_controller.getLeftTrigger(), m_shooter)
    // );
    
    m_intake.setDefaultCommand(
      new IntakeCommand(
        () -> m_controller.getLeftTrigger(), m_intake)
    );

    m_indexer.setDefaultCommand(
      new IndexCommand(
        () -> m_controller.getRightTrigger(), m_indexer)
    );


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_controller.xButton.whenHeld(new IntakePistons(m_intake));
  }

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
