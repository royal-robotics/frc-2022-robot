package frc.robot.commands;

import frc.robot.input.XboxController;
import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoClimbCommand extends CommandBase {
    private final ClimberSubsystem m_subsystem;

    public AutoClimbCommand(ClimberSubsystem subsystem
          ){

        m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
