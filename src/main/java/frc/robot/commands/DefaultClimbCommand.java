package frc.robot.commands;

import frc.robot.input.XboxController;
import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultClimbCommand extends CommandBase {
    private final ClimberSubsystem m_subsystem;
    private final DoubleSupplier m_climberAngleSupplier;
    private final DoubleSupplier m_climberSupplier;

    public DefaultClimbCommand(ClimberSubsystem subsystem,
            DoubleSupplier climberAngleSupplier,
            DoubleSupplier climberSupplier){

        m_subsystem = subsystem;
        m_climberAngleSupplier = climberAngleSupplier;
        m_climberSupplier = climberSupplier;

        addRequirements(subsystem);
    }

    public DefaultClimbCommand(ClimberSubsystem subsystem, XboxController controller) {
        this(
            subsystem,
            ()-> -controller.getRightX().get(),
            ()-> -controller.getRightY().get());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double climber = m_climberSupplier.getAsDouble();
        double climberAngle = m_climberAngleSupplier.getAsDouble();
        m_subsystem.setMotorStates(climber, climberAngle);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setMotorStates(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
