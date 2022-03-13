package frc.robot.autonomous.modes;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutoModeBase;
import frc.robot.commands.AutoFollowCommand;
import frc.robot.commands.AutoMoveShooter;
import frc.robot.commands.AutoPickupCommand;
import frc.robot.commands.AutoRotateCommand;
import frc.robot.commands.AutoShootCommand;

public class ThreeBallAuto extends AutoModeBase {
    public ThreeBallAuto(RobotContainer robotContainer) {
        super("ThreeBall");

        var drivetrainSubsystem = robotContainer.drivetrainSubsystem;
        var shooterSubsystem = robotContainer.shooterSubsystem;
        //this.addCommands(new WaitCommand(.75));
        this.addCommands(new AutoMoveShooter(shooterSubsystem, -21));
        this.addCommands(new AutoShootCommand(shooterSubsystem, 2500));
        this.addCommands(new AutoPickupCommand(shooterSubsystem));
        this.addCommands(new AutoFollowCommand(drivetrainSubsystem, "StraightPath"));
        this.addCommands(new WaitCommand(1));
        this.addCommands(new AutoMoveShooter(shooterSubsystem, -21));
        this.addCommands(new AutoShootCommand(shooterSubsystem, 2800));
        this.addCommands(new AutoPickupCommand(shooterSubsystem));
        this.addCommands(new AutoFollowCommand(drivetrainSubsystem, "DiagonalPath"));
        this.addCommands(new WaitCommand(1));
        this.addCommands(new AutoRotateCommand(drivetrainSubsystem));
        this.addCommands(new AutoShootCommand(shooterSubsystem, 2800));
    }
}