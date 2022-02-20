package frc.robot.commands;


import frc.robot.input.XboxController;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultShootCommand extends CommandBase {
    private final ShooterSubsystem m_subsystem;
    private final DoubleSupplier m_shooterAngleSupplier;
    private final BooleanSupplier m_intake;
    private final BooleanSupplier m_outtake;
    private final BooleanSupplier m_shoot;

    public DefaultShootCommand(ShooterSubsystem subsystem,
            DoubleSupplier shooterAngleSupplier,
            BooleanSupplier intake,
            BooleanSupplier outtake,
            BooleanSupplier shoot){

        m_subsystem = subsystem;
        m_shooterAngleSupplier = shooterAngleSupplier;
        m_intake = intake;
        m_outtake = outtake;
        m_shoot = shoot;

        addRequirements(subsystem);
    }

    public DefaultShootCommand(ShooterSubsystem subsystem, XboxController controller) {
        this(
            subsystem,
            ()-> -controller.getLeftY().get(),
            ()-> controller.getA().get(),
            ()-> controller.getY().get(),
            ()-> controller.getX().get());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double shooterAngle = m_shooterAngleSupplier.getAsDouble();
        double shooterWheels = (m_shoot.getAsBoolean()) ? 0.25 : 0;

        boolean in = m_intake.getAsBoolean();
        boolean out = m_outtake.getAsBoolean();
        double intakeWheels = 0;

        if(in && out){
            intakeWheels = 0;
        }else if(in){
            intakeWheels = 0.25;
        }else if(out){
            intakeWheels = -0.25;
        }

        m_subsystem.setMotorStates(shooterAngle, shooterWheels, intakeWheels);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setMotorStates(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
