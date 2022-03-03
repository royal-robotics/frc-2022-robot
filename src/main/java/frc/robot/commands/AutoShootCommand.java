package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix.time.StopWatch;

public class AutoShootCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;
    private final double m_angleSetpoint;
    private boolean end = false;
    private final double m_wheelSpeed;
    private final StopWatch m_stopWatch = new StopWatch();

    public AutoShootCommand(ShooterSubsystem shooterSubsystem, double angleSetpoint, double wheelSpeed) {
        m_shooterSubsystem = shooterSubsystem;
        m_angleSetpoint = angleSetpoint;
        m_wheelSpeed = wheelSpeed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.setAngleSetpoint(m_angleSetpoint);
        //m_shooterSubsystem.setSpeedSetpoint(m_wheelSpeed);
    }

    @Override
    public void execute() {
        if(m_shooterSubsystem.atAngleSetpoint()){
            m_shooterSubsystem.setSpeedSetpoint(m_wheelSpeed);
        }
        if(m_shooterSubsystem.atSpeedSetpoint()){
            m_stopWatch.start();
        }
        if (m_shooterSubsystem.atAngleSetpoint() && m_shooterSubsystem.atSpeedSetpoint() && m_stopWatch.getDurationMs() > 250) {
            m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kReverse);
            if(!end){
                m_stopWatch.start();
                end = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setSpeedSetpoint(0);
        m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kForward);
    }

    @Override
    public boolean isFinished() {
        return end && m_stopWatch.getDurationMs() > 250;
    }
}
