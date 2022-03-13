package frc.robot.autonomous.modes;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutoModeBase;
import frc.robot.commands.AutoFollowCommand;

public class TestAuto extends AutoModeBase {
    public TestAuto(RobotContainer robotContainer) {
        super("Test");

        var drivetrainSubsystem = robotContainer.drivetrainSubsystem;
        this.addCommands(new WaitCommand(0.5));
        this.addCommands(new AutoFollowCommand(drivetrainSubsystem, "MeterPath"));

        addRequirements(robotContainer.drivetrainSubsystem);
    }


}
