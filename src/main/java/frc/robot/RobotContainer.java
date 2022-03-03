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

    public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private final PickupCommand m_pickupCommand = new PickupCommand(shooterSubsystem);
    private final ShootCommand m_shootCommand = new ShootCommand(shooterSubsystem, drivetrainSubsystem, m_operator, m_controller);

    public RobotContainer() {
        drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(drivetrainSubsystem, m_controller));
        climberSubsystem.setDefaultCommand(new DefaultClimbCommand(climberSubsystem, m_operator));
        shooterSubsystem.setDefaultCommand(new DefaultShootCommand(shooterSubsystem, m_operator));


        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // m_controller.getA().whenPressed(
        //     () ->  m_drivetrainSubsystem.zeroGyroscope()
        // );
        m_controller.getT1().whenPressed(
            () -> drivetrainSubsystem.zeroGyroscope()
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
            climberSubsystem.resetEncoder();
        }
    }

    /* //moved to automonous folder. RIP
    public Command getAutonomousCommand() {
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
        return new SequentialCommandGroup(new AutoShootCommand(shooterSubsystem, -21, 2900), new AutoPickupCommand(shooterSubsystem), pathCommand, new InstantCommand(() -> drivetrainSubsystem.drive(new ChassisSpeeds()), drivetrainSubsystem));
    }
    */
}
