package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix.time.StopWatch;

public class AutoMoveShooter extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;
    private final double m_angleSetpoint;
    private final StopWatch m_stopWatch = new StopWatch();
    private boolean m_gotToSetpoint = false;


    public AutoMoveShooter(ShooterSubsystem shooterSubsystem, double angleSetpoint) {
        m_shooterSubsystem = shooterSubsystem;
        m_angleSetpoint = angleSetpoint;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.setSpeedSetpoint(0);
        m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kForward);
        m_shooterSubsystem.setAngleSetpoint(m_angleSetpoint);
    }

    @Override
    public void execute() {
        if(m_shooterSubsystem.atAngleSetpoint() && !m_gotToSetpoint){
            m_gotToSetpoint = true;
            m_stopWatch.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return m_gotToSetpoint && m_stopWatch.getDurationMs() > 250;
    }
}
