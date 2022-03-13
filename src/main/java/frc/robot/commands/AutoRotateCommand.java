package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AutoRotateCommand extends CommandBase {
    private final DrivetrainSubsystem m_subsystem;

    public AutoRotateCommand(
            DrivetrainSubsystem subsystem) {
        m_subsystem = subsystem;
      
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
       m_subsystem.drive(new ChassisSpeeds(0,0, Math.PI/2));
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
