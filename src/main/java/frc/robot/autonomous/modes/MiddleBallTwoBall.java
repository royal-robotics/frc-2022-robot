package frc.robot.autonomous.modes;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutoModeBase;
import frc.robot.commands.AutoFollowCommand;
import frc.robot.commands.AutoMoveShooter;
import frc.robot.commands.AutoPickupCommand;
import frc.robot.commands.AutoRotateCommand;
import frc.robot.commands.AutoShootCommand;

public class MiddleBallTwoBall extends AutoModeBase {
    public MiddleBallTwoBall(RobotContainer robotContainer) {
        super("MiddleBallTwoBall");

        var drivetrainSubsystem = robotContainer.drivetrainSubsystem;
        var shooterSubsystem = robotContainer.shooterSubsystem;
        //this.addCommands(new WaitCommand(.75));
        //this.addCommands(new AutoMoveShooter(shooterSubsystem, -21));
        this.addCommands(new AutoShootCommand(shooterSubsystem, 2500));
        //this.addCommands(new ParallelCommandGroup(new AutoPickupCommand(shooterSubsystem), new AutoFollowCommand(drivetrainSubsystem, "MiddleBallPathRedTest", 4, 2)));
        this.addCommands(new AutoPickupCommand(shooterSubsystem));
        this.addCommands(new AutoFollowCommand(drivetrainSubsystem, "MiddleBallPathRedTest", 4, 2));
        //this.addCommands(new WaitCommand(1));
        this.addCommands(new AutoMoveShooter(shooterSubsystem, -21));
        this.addCommands(new AutoShootCommand(shooterSubsystem, 3200));
    }
}
