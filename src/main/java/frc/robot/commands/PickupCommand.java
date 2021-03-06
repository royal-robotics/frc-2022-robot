package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PickupCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;

    public PickupCommand(ShooterSubsystem subsystem) {
        m_shooterSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.setAngleSetpoint(m_shooterSubsystem.BOTTOM_ANGLE);
        m_shooterSubsystem.setMotorStates(-4000, -0.75);
        m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kReverse, DoubleSolenoid.Value.kForward);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setAngleSetpoint(20);
        m_shooterSubsystem.setMotorStates(0, 0);
        m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kForward);
    }
}
