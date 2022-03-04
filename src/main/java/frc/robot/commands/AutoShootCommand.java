package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix.time.StopWatch;

public class AutoShootCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;
    private boolean started = false;
    private boolean end = false;
    private boolean atWheelSpeed = false;
    private boolean kickerTimeout = false;
    private final double m_wheelSpeed;
    private StopWatch kickerStopWatch = null;
    private StopWatch wheelSpeedTimer = null;
    private boolean holdWheelSpeed = false;


    public AutoShootCommand(ShooterSubsystem shooterSubsystem, double wheelSpeed) {
        m_shooterSubsystem = shooterSubsystem;
        m_wheelSpeed = wheelSpeed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.setSpeedSetpoint(m_wheelSpeed);
    }

    @Override
    public void execute() {
        if(m_shooterSubsystem.atAngleSetpoint()&& !holdWheelSpeed){
            holdWheelSpeed = true;
            wheelSpeedTimer = new StopWatch();
            wheelSpeedTimer.start();
        }

        if(m_shooterSubsystem.atAngleSetpoint() && !atWheelSpeed && wheelSpeedTimer != null &&wheelSpeedTimer.getDurationMs() > 500){
            atWheelSpeed = true;
            kickerStopWatch = new StopWatch();
            kickerStopWatch.start();
            m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kReverse);
        }

        if(kickerStopWatch != null && kickerStopWatch.getDurationMs() > 250 && !kickerTimeout){
            kickerTimeout = true;
            m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kForward);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return kickerTimeout;
    }
}
