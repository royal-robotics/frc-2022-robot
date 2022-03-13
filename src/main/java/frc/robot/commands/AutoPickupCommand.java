package frc.robot.commands;

import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoPickupCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;
    private final StopWatch m_stopWatch = new StopWatch();

    public AutoPickupCommand(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.setAngleSetpoint(m_shooterSubsystem.pickupAngle);
        m_shooterSubsystem.setMotorStates(-4000, -0.5);
        m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kReverse, DoubleSolenoid.Value.kForward);
    }

    @Override
    public void execute() {
        if(m_shooterSubsystem.atAngleSetpoint()){
            m_stopWatch.start();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return m_shooterSubsystem.atAngleSetpoint() ; //&& m_stopWatch.getDurationMs()>1000;
    }
}
