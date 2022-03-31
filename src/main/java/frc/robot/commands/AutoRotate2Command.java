package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class AutoRotate2Command extends CommandBase {
    private final DrivetrainSubsystem m_subsystem;
    private final double m_targetAngle;
    private Timer m_targetTime;

    public AutoRotate2Command(
            DrivetrainSubsystem subsystem,
            double targetAngle) {
        m_subsystem = subsystem;
        m_targetAngle = targetAngle;
        m_targetTime = new Timer();

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_targetTime.reset();
    }

    @Override
    public void execute() {
        double angle = m_subsystem.getGyroscopeRotation().getDegrees();
        double offset = m_targetAngle - angle;
        double power = offset * 0.125;
        if (power > Math.PI) {
            power = Math.PI;
        } else if (power < -Math.PI) {
            power = -Math.PI;
        }
        m_subsystem.drive(new ChassisSpeeds(0, 0, power));
        if (offset > -5.0 && offset < 5.0) {
            m_targetTime.start();
        } else {
            m_targetTime.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return m_targetTime.get() > 0.25;
    }
}
