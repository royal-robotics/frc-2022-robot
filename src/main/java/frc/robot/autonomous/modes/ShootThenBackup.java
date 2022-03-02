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

        PathPlannerTrajectory examplePath = PathPlanner.loadPath("TurnPath", DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 5);
        //PIDController x_control = new PIDController(0.5, 0, 0);
        //PIDController y_control = new PIDController(0.5, 0, 0);
        PIDController x_control = new PIDController(0, 0, 0);
        PIDController y_control = new PIDController(0, 0, 0);
        ProfiledPIDController angle_control = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 5));
        angle_control.enableContinuousInput(-Math.PI, Math.PI);
        drivetrainSubsystem.resetPose(examplePath.getInitialPose());
        drivetrainSubsystem.setGyroscope(examplePath.getInitialPose().getRotation().getDegrees());
        PPSwerveControllerCommand pathCommand = new PPSwerveControllerCommand(examplePath,() -> drivetrainSubsystem.getPose(),drivetrainSubsystem.getKinematics(),x_control,y_control, angle_control, (SwerveModuleState[] states) -> drivetrainSubsystem.setModuleStates(states),drivetrainSubsystem);
        this.addCommands(pathCommand);
        this.addCommands(new InstantCommand(() -> drivetrainSubsystem.drive(new ChassisSpeeds()), drivetrainSubsystem));
    }
}
