package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoFollowCommand extends SequentialCommandGroup {

    public AutoFollowCommand(DrivetrainSubsystem drivetrainSubsystem, String pathName, double maxV, double maxA) {
        var trajectory = PathPlanner.loadPath(pathName, maxV, maxA);

        // Reset gyro and pose to match path
        this.addCommands(new InstantCommand(()-> {
            drivetrainSubsystem.resetPose(trajectory.getInitialPose());
            drivetrainSubsystem.setGyroscope(trajectory.getInitialPose().getRotation().getDegrees());
        }));
        // Wait a tiny bit
        this.addCommands(new WaitCommand(0.1));

        // Follow path
        PIDController x_control = new PIDController(0.00, 0, 0);
        PIDController y_control = new PIDController(0.00, 0, 0);
        ProfiledPIDController angle_control = new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(maxV, maxA)); //DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 5
        angle_control.enableContinuousInput(-Math.PI, Math.PI);
        PPSwerveControllerCommand pathCommand = new PPSwerveControllerCommand(
            trajectory,
            () -> drivetrainSubsystem.getPose(),
            drivetrainSubsystem.getKinematics(),
            x_control,y_control,
            angle_control,
            (SwerveModuleState[] states) -> drivetrainSubsystem.setModuleStates(states),drivetrainSubsystem
        );
        this.addCommands(pathCommand);
        // Stop drivetrain
        this.addCommands(new InstantCommand(() -> drivetrainSubsystem.drive(new ChassisSpeeds()), drivetrainSubsystem));
    }
}
