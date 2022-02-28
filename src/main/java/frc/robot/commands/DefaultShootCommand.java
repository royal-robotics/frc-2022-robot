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
    private final BooleanSupplier m_intakeSupplier;
    private final BooleanSupplier m_outtakeSupplier;
    private final BooleanSupplier m_reverseWheelsSupplier;
    private final BooleanSupplier m_extendIntakeSupplier;
    private final BooleanSupplier m_kickerSupplier;

    public DefaultShootCommand(ShooterSubsystem subsystem,
            DoubleSupplier shooterAngleSupplier,
            DoubleSupplier shooterWheelsSupplier,
            BooleanSupplier intakeSupplier,
            BooleanSupplier outtakeSupplier,
            BooleanSupplier reverseWheelsSupplier,
            BooleanSupplier extendIntakeSupplier,
            BooleanSupplier kickerSupplier){

        m_subsystem = subsystem;
        m_shooterAngleSupplier = shooterAngleSupplier;
        m_shooterWheelsSupplier = shooterWheelsSupplier;
        m_intakeSupplier = intakeSupplier;
        m_outtakeSupplier = outtakeSupplier;
        m_reverseWheelsSupplier = reverseWheelsSupplier;
        m_extendIntakeSupplier = extendIntakeSupplier;
        m_kickerSupplier = kickerSupplier;

        addRequirements(subsystem);
    }

    public DefaultShootCommand(ShooterSubsystem subsystem, XboxController controller) {
        this(
            subsystem,
            ()-> -controller.getLeftY().get(),
            ()-> controller.getLeftTrigger().get(),
            ()-> controller.getX().get(),
            ()-> controller.getY().get(),
            ()-> controller.getStart().get(),
            ()-> controller.getRightBumper().get(),
            ()-> controller.getLeftBumper().get());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double shooterWheels = m_reverseWheelsSupplier.getAsBoolean() ?
            m_shooterWheelsSupplier.getAsDouble() :
            -m_shooterWheelsSupplier.getAsDouble();

        boolean in = m_intakeSupplier.getAsBoolean();
        boolean out = m_outtakeSupplier.getAsBoolean();
        double intakeWheels = 0;

        if(in && out){
            intakeWheels = 0;
        }else if(in){
            intakeWheels = -0.5;
            shooterWheels = -1.0;
        }else if(out){
            intakeWheels = 0.5;
            shooterWheels = 1.0;
        }

        m_subsystem.setMotorStates(shooterWheels, intakeWheels);

        double setpointChange = m_shooterAngleSupplier.getAsDouble() * 2;
        double newSetpoint = m_subsystem.getAngle() + setpointChange;
        m_subsystem.setAngleSetpoint(newSetpoint);

        DoubleSolenoid.Value extendIntake = m_extendIntakeSupplier.getAsBoolean() ?
            DoubleSolenoid.Value.kReverse :
            DoubleSolenoid.Value.kForward;
        DoubleSolenoid.Value kicker = m_kickerSupplier.getAsBoolean() ?
            DoubleSolenoid.Value.kReverse :
            DoubleSolenoid.Value.kForward;

        m_subsystem.setSolenoidStates(extendIntake, kicker);
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
