package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {
    private final DriveTrain m_driveTrain;
    private final DoubleSupplier m_speed;
    private final DoubleSupplier m_rotate;
    
    public TankDrive(DoubleSupplier speed, DoubleSupplier rotate, DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
        m_speed = speed;
        m_rotate = rotate;
        addRequirements(m_driveTrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_driveTrain.set(m_speed.getAsDouble(), m_rotate.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_driveTrain.stop();
    }
}
