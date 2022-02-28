package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;
    private final double m_angleSetpoint;
    private boolean end = false;
    private int endLoops = 0;

    public AutoShootCommand(ShooterSubsystem shooterSubsystem, double angleSetpoint) {
        m_shooterSubsystem = shooterSubsystem;
        m_angleSetpoint = angleSetpoint;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.setAngleSetpoint(m_angleSetpoint);
        m_shooterSubsystem.setSpeedSetpoint(2900);
    }

    @Override
    public void execute() {
        if (m_shooterSubsystem.atAngleSetpoint() && m_shooterSubsystem.atSpeedSetpoint()) {
            m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kReverse);
            end = true;
        }
        if (end) {
            endLoops++;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setSpeedSetpoint(0);
        m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kForward);
    }

    @Override
    public boolean isFinished() {
        return endLoops > 30;
    }
}
