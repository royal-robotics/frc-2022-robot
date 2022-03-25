package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoRotateCommand extends CommandBase {
    private final DrivetrainSubsystem m_subsystem;
    private final double m_angle;
    private double targetAngle;

    public AutoRotateCommand(
            DrivetrainSubsystem subsystem,
            double angle) {
        m_subsystem = subsystem;
        m_angle = angle;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        targetAngle = m_angle + m_subsystem.getGyroscopeRotation().getDegrees();
    }

    @Override
    public void execute() {
       m_subsystem.drive(new ChassisSpeeds(0,0, Math.PI));
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return m_subsystem.getGyroscopeRotation().getDegrees() > targetAngle;
    }

}
