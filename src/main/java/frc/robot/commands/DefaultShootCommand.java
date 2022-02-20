package frc.robot.commands;


import frc.robot.input.XboxController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultShootCommand extends CommandBase {
    private final ShooterSubsystem m_subsystem;
    private final DoubleSupplier m_LeftXSupplier;
    private final BooleanSupplier m_intake;
    private final BooleanSupplier m_outtake;
    private final BooleanSupplier m_shoot;


    public DefaultShootCommand(ShooterSubsystem subsystem,
            DoubleSupplier LeftXSupplier,
            BooleanSupplier intake,
            BooleanSupplier outtake,
            BooleanSupplier shoot){

        m_subsystem = subsystem;
        m_LeftXSupplier = LeftXSupplier;
        m_intake = intake;
        m_outtake = outtake;
        m_shoot = shoot;
        
        addRequirements(subsystem);
    }

    public DefaultShootCommand(ShooterSubsystem subsystem, XboxController controller) {
        this(
            subsystem,
            ()-> -controller.getLeftX().get(),
            ()-> controller.getA().get(),
            ()-> controller.getY().get(),
            ()-> controller.getX().get());
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
        double[] actions = new double[4];
        boolean in = m_intake.getAsBoolean();
        boolean out = m_outtake.getAsBoolean();
        double intakeWheels = 0;
        actions[0] = m_LeftXSupplier.getAsDouble();
        actions[1] = (m_shoot.getAsBoolean()) ? 0.25 : 0;
        actions[2] = actions[1];

        if(in && out){
            intakeWheels = 0;
        }else if(in){
            intakeWheels = 0.25;
        }else if(out){
            intakeWheels = -0.25;
        }

        actions[3] = intakeWheels;
        m_subsystem.setModuleStates(actions);
    }

    @Override
    public void end(boolean interrupted) {
        double[] zeroes = {0,0,0,0};
        m_subsystem.setModuleStates(zeroes);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
