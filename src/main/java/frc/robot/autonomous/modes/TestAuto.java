package frc.robot.autonomous.modes;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutoModeBase;
import frc.robot.commands.AutoFollowCommand;
import frc.robot.commands.AutoMoveShooter;
import frc.robot.commands.AutoPickupCommand;
import frc.robot.commands.AutoShootCommand;

public class TestAuto extends AutoModeBase {
    public TestAuto(RobotContainer robotContainer) {
        super("Test");

        var drivetrainSubsystem = robotContainer.drivetrainSubsystem;
        this.addCommands(new AutoFollowCommand(drivetrainSubsystem, "SquarePath"));

    }

    
}
