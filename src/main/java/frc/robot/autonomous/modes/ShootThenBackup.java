package frc.robot.autonomous.modes;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutoModeBase;
import frc.robot.commands.AutoFollowCommand;
import frc.robot.commands.AutoMoveShooter;
import frc.robot.commands.AutoPickupCommand;
import frc.robot.commands.AutoShootCommand;

public class ShootThenBackup extends AutoModeBase {
    public ShootThenBackup(RobotContainer robotContainer) {
        super("Shoot Then Backup");

        var drivetrainSubsystem = robotContainer.drivetrainSubsystem;
        var shooterSubsystem = robotContainer.shooterSubsystem;
        //this.addCommands(new WaitCommand(.75));
        //this.addCommands(new AutoMoveShooter(shooterSubsystem, -21));
        this.addCommands(new AutoShootCommand(shooterSubsystem, 2500));
        //this.addCommands(new AutoPickupCommand(shooterSubsystem));
        this.addCommands(new AutoFollowCommand(drivetrainSubsystem, "StraightPath", 4, 2));
        //this.addCommands(new WaitCommand(0.25));
        //this.addCommands(new AutoMoveShooter(shooterSubsystem, shooterSubsystem.TOP_ANGLE));
        //this.addCommands(new AutoShootCommand(shooterSubsystem, 2900));
        this.addCommands(new AutoFollowCommand(drivetrainSubsystem, "SmallPath", 4, 2));
    }
}
