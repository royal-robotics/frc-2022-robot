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
    private final BooleanSupplier m_angleExtendSupplier;
    private final BooleanSupplier m_angleRetractSupplier;
    private double lastDistance;

    public DefaultClimbCommand(ClimberSubsystem subsystem,
            DoubleSupplier climberAngleSupplier,
            DoubleSupplier climberSupplier,
            BooleanSupplier enableClimberAngle,
            BooleanSupplier angleExtendSupplier,
            BooleanSupplier angleRetractSupplier){

        m_subsystem = subsystem;
        m_climberAngleSupplier = climberAngleSupplier;
        m_climberSupplier = climberSupplier;
        m_enableClimberAngle = enableClimberAngle;
        m_angleExtendSupplier = angleExtendSupplier;
        m_angleRetractSupplier = angleRetractSupplier;

        addRequirements(subsystem);
    }

    public DefaultClimbCommand(ClimberSubsystem subsystem, XboxController controller) {
        this(
            subsystem,
            ()-> -controller.getLeftX().get(),
            ()-> controller.getRightY().get(),
            ()-> controller.getRightBumper().get(),
            ()-> controller.getDpadLeft().get(),
            ()-> controller.getDpadRight().get());
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

        if (m_angleExtendSupplier.getAsBoolean()) {
            m_subsystem.setAngleSetpoint(80);
        } else if (m_angleRetractSupplier.getAsBoolean()) {
            m_subsystem.setAngleSetpoint(65);
        }

        double distanceSetpointChange = m_climberSupplier.getAsDouble() * 2;
        if (distanceSetpointChange != 0) {
            lastDistance = m_subsystem.getDistance() + distanceSetpointChange;
            m_subsystem.setDistanceSetpoint(lastDistance);
        } else {
            m_subsystem.setDistanceSetpoint(lastDistance);
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
