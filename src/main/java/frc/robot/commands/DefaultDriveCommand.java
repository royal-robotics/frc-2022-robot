package frc.robot.commands;

import frc.robot.input.StickController;
import frc.robot.input.XboxController;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends DriveCommandBase {

    public DefaultDriveCommand(DrivetrainSubsystem subsystem, StickController controller) {
    //public DefaultDriveCommand(DrivetrainSubsystem subsystem, XboxController controller) {
        super(
            subsystem,
            ()-> -controller.getForwardAxis().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> -controller.getStrafeAxis().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> -controller.getRotateAxis().get(0.2, true, false) * 0.5 * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            ()-> controller.getTrigger().get());
            /*()-> controller.getLeftY().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> controller.getLeftX().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> controller.getRightX().get() * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            ()-> controller.getRightTrigger().get() > 0.5);*/
            //deadband used to be 0.4
    }
}
