package frc.robot.commands;

import frc.robot.input.XboxController;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultShootCommand extends CommandBase {
    private final ShooterSubsystem m_subsystem;

    private final DoubleSupplier m_shooterAngleSupplier;
    private final DoubleSupplier m_shooterWheelsSupplier;
    private final BooleanSupplier m_disableShooterAngle;
    private final BooleanSupplier m_kickerSupplier;
    private final BooleanSupplier m_shortShotSupplier;
    private final BooleanSupplier m_midShotSupplier;
    private final BooleanSupplier m_longShotSupplier;

    public DefaultShootCommand(ShooterSubsystem subsystem,
            DoubleSupplier shooterAngleSupplier,
            DoubleSupplier shooterWheelsSupplier,
            BooleanSupplier disableShooterSupplier,
            BooleanSupplier kickerSupplier,
            BooleanSupplier shortShotSupplier,
            BooleanSupplier midShotSupplier,
            BooleanSupplier longShotSupplier){

        m_subsystem = subsystem;
        m_shooterAngleSupplier = shooterAngleSupplier;
        m_shooterWheelsSupplier = shooterWheelsSupplier;
        m_disableShooterAngle = disableShooterSupplier;
        m_kickerSupplier = kickerSupplier;
        m_shortShotSupplier = shortShotSupplier;
        m_midShotSupplier = midShotSupplier;
        m_longShotSupplier = longShotSupplier;

        addRequirements(subsystem);
    }

    public DefaultShootCommand(ShooterSubsystem subsystem, XboxController controller) {
        this(
            subsystem,
            ()-> -controller.getLeftY().get(),
            ()-> controller.getLeftTrigger().get(),
            ()-> controller.getRightBumper().get(),
            ()-> controller.getLeftBumper().get(),
            ()-> controller.getY().get(),
            ()-> controller.getX().get(),
            ()-> controller.getA().get());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double shooterWheels = -m_shooterWheelsSupplier.getAsDouble() * m_subsystem.m_speedEntry.getDouble(m_subsystem.RPM_TOP);
        if (shooterWheels != 0) {
            if (m_shortShotSupplier.getAsBoolean()) {
                shooterWheels = 2600;
            } else if (m_midShotSupplier.getAsBoolean()) {
                shooterWheels = 3400;
            } else if (m_longShotSupplier.getAsBoolean()) {
                shooterWheels = 4200;
            }
        }
        m_subsystem.setMotorStates(shooterWheels, 0);

        if(m_disableShooterAngle.getAsBoolean()==false){
            double setpointChange = m_shooterAngleSupplier.getAsDouble() * 2;
            double newSetpoint = m_subsystem.getAngleSetpoint() + setpointChange;

            m_subsystem.setAngleSetpoint(newSetpoint);
        }

        DoubleSolenoid.Value kicker = m_kickerSupplier.getAsBoolean() ?
            DoubleSolenoid.Value.kReverse :
            DoubleSolenoid.Value.kForward;

        m_subsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, kicker);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setMotorStates(0, 0);
        m_subsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kForward);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
