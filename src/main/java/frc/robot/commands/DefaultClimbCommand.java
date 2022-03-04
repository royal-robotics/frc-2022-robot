package frc.robot.commands;

import frc.robot.input.XboxController;
import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultClimbCommand extends CommandBase {
    private final ClimberSubsystem m_subsystem;

    private final DoubleSupplier m_climberAngleSupplier;
    private final DoubleSupplier m_climberSupplier;
    private final BooleanSupplier m_enableClimberAngle;

    public DefaultClimbCommand(ClimberSubsystem subsystem,
            DoubleSupplier climberAngleSupplier,
            DoubleSupplier climberSupplier,
            BooleanSupplier enableClimberAngle){

        m_subsystem = subsystem;
        m_climberAngleSupplier = climberAngleSupplier;
        m_climberSupplier = climberSupplier;
        m_enableClimberAngle = enableClimberAngle;

        addRequirements(subsystem);
    }

    public DefaultClimbCommand(ClimberSubsystem subsystem, XboxController controller) {
        this(
            subsystem,
            ()-> -controller.getLeftX().get(),
            ()-> controller.getRightY().get(),
            ()-> controller.getRightBumper().get());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(m_enableClimberAngle.getAsBoolean()){
            
        double angleSetpointChange = m_climberAngleSupplier.getAsDouble() * 15;
            if (angleSetpointChange != 0) {
                double newAngleSetpoint = m_subsystem.getAngle() + angleSetpointChange;
                m_subsystem.setAngleSetpoint(newAngleSetpoint);
            }
        }
        

        double distanceSetpointChange = m_climberSupplier.getAsDouble() * 2;
        if (distanceSetpointChange != 0) {
            double newDistanceSetpoint = m_subsystem.getDistance() + distanceSetpointChange;
            m_subsystem.setDistanceSetpoint(newDistanceSetpoint);
        }

        double climber = m_climberSupplier.getAsDouble();
        double climberAngle = m_climberAngleSupplier.getAsDouble() * 0.25;
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
