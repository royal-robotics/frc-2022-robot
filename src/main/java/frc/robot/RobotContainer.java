// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.input.StickController;
import frc.robot.input.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.pathplanner.lib.commands.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private static final int PRIMARY_CONTROLLER_PORT = 0;
    private static final int SECONDARY_CONTROLLER_PORT = 1;

    // private final XboxController m_controller = new XboxController(PRIMARY_CONTROLLER_PORT);
    private final StickController m_controller = new StickController(PRIMARY_CONTROLLER_PORT);
    private final XboxController m_operator = new XboxController(SECONDARY_CONTROLLER_PORT);

    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

    private final PickupCommand m_pickupCommand = new PickupCommand(m_shooterSubsystem);
    private final ShootCommand m_shootCommand = new ShootCommand(m_shooterSubsystem, m_drivetrainSubsystem, m_operator, m_controller);

    public RobotContainer() {
        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem, m_controller));
        m_climberSubsystem.setDefaultCommand(new DefaultClimbCommand(m_climberSubsystem, m_operator));
        m_shooterSubsystem.setDefaultCommand(new DefaultShootCommand(m_shooterSubsystem, m_operator));


        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // m_controller.getA().whenPressed(
        //     () ->  m_drivetrainSubsystem.zeroGyroscope()
        // );
        m_controller.getT1().whenPressed(
            () -> m_drivetrainSubsystem.zeroGyroscope()
        );
        m_operator.getA().whenPressed(
            () -> m_pickupCommand.schedule()
        );
        m_operator.getA().whenReleased(
            () -> m_pickupCommand.cancel()
        );
        m_operator.getB().whenPressed(
            () -> m_shootCommand.schedule()
        );
        m_operator.getB().whenReleased(
            () -> m_shootCommand.cancel()
        );
        if(m_operator.getRightTrigger().get()>0){
            m_climberSubsystem.resetEncoder();
        }
    }

    public Command getAutonomousCommand() {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("TurnPath", DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 5);
        //PIDController x_control = new PIDController(0.5, 0, 0);
        //PIDController y_control = new PIDController(0.5, 0, 0);
        PIDController x_control = new PIDController(0, 0, 0);
        PIDController y_control = new PIDController(0, 0, 0);
        ProfiledPIDController angle_control = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 5));
        angle_control.enableContinuousInput(-Math.PI, Math.PI);
        m_drivetrainSubsystem.resetPose(examplePath.getInitialPose());
        m_drivetrainSubsystem.setGyroscope(examplePath.getInitialPose().getRotation().getDegrees());
        PPSwerveControllerCommand pathCommand = new PPSwerveControllerCommand(examplePath,() -> m_drivetrainSubsystem.getPose(),m_drivetrainSubsystem.getKinematics(),x_control,y_control, angle_control, (SwerveModuleState[] states) -> m_drivetrainSubsystem.setModuleStates(states),m_drivetrainSubsystem);
        return new SequentialCommandGroup(new AutoShootCommand(m_shooterSubsystem, -21), new AutoPickupCommand(m_shooterSubsystem), pathCommand, new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds()), m_drivetrainSubsystem));
    }
}
