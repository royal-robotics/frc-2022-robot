package frc.robot.commands;

import frc.robot.input.StickController;
import frc.robot.input.XboxController;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends DriveCommandBase {

    public DefaultDriveCommand(DrivetrainSubsystem subsystem, StickController controller) {
        super(
            subsystem,
            ()-> controller.getForwardAxis().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> controller.getStrafeAxis().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> controller.getRotateAxis().get() * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    }
}
