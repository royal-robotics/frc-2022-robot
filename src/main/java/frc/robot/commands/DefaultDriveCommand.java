package frc.robot.commands;

import frc.robot.input.StickController;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends DriveCommandBase {

    public DefaultDriveCommand(DrivetrainSubsystem subsystem, StickController controller) {
        super(
            subsystem,
            ()-> -controller.getForwardAxis().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> -controller.getStrafeAxis().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> -controller.getRotateAxis().get(0.2, true, false) * 0.5 * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            ()-> controller.getTrigger().get());
            //deadband used to be 0.4
    }
}
