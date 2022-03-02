package frc.robot.autonomous.modes;

import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlanner.*;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutoModeBase;
import frc.robot.commands.AutoFollowCommand;
import frc.robot.commands.AutoPickupCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ShootThenBackup extends AutoModeBase {
    public ShootThenBackup(RobotContainer robotContainer) {
        super("Shoot Then Backup");

        var drivetrainSubsystem = robotContainer.drivetrainSubsystem;
        var shooterSubsystem = robotContainer.shooterSubsystem;

        this.addCommands(new AutoShootCommand(shooterSubsystem, -21));
        this.addCommands(new AutoPickupCommand(shooterSubsystem));
        this.addCommands(new AutoFollowCommand(drivetrainSubsystem, "TurnPath"));
    }
}
