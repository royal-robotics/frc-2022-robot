package frc.robot.commands;

import frc.robot.input.XboxController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultClimbCommand extends CommandBase {
    private final ClimberSubsystem m_subsystem;
    private final DoubleSupplier m_RightXSupplier;
    private final DoubleSupplier m_RightYSupplier;
    private final DoubleSupplier m_LeftYSupplier;

    public DefaultClimbCommand(ClimberSubsystem subsystem,
            DoubleSupplier RightXSupplier,
            DoubleSupplier RightYSupplier,
            DoubleSupplier LeftYSupplier){

        m_subsystem = subsystem;
        m_RightXSupplier = RightXSupplier;
        m_RightYSupplier = RightYSupplier;
        m_LeftYSupplier = LeftYSupplier;

        addRequirements(subsystem);
    }

    public DefaultClimbCommand(ClimberSubsystem subsystem, XboxController controller) {
        this(
            subsystem,
            ()-> -controller.getRightX().get(),
            ()-> -controller.getRightY().get(),
            ()-> -controller.getLeftY().get(0.4));

        /*
        super(
            subsystem,
            ()-> -controller.getForwardAxis().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> -controller.getStrafeAxis().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> -controller.getRotateAxis().get(0.4) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
            */
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double[] actions = new double[3];
        actions[0] = m_RightXSupplier.getAsDouble();
        actions[1] = actions[0];
        actions[2] = m_RightYSupplier.getAsDouble();
        m_subsystem.setModuleStates(actions);
    }

    @Override
    public void end(boolean interrupted) {
        double[] zeroes = {0,0,0};
        m_subsystem.setModuleStates(zeroes);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
