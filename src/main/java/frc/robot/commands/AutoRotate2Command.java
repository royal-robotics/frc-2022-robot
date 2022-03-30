package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoRotate2Command extends CommandBase {
    private final DrivetrainSubsystem m_subsystem;
    private final double m_targetAngle;

    public AutoRotate2Command(
            DrivetrainSubsystem subsystem,
            double targetAngle) {
        m_subsystem = subsystem;
        m_targetAngle = targetAngle - 27;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
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
        return m_subsystem.getGyroscopeRotation().getDegrees() > m_targetAngle;
    }

}
