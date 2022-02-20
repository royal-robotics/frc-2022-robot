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

    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

   // private final XboxController m_controller = new XboxController(PRIMARY_CONTROLLER_PORT);
   private final StickController m_controller = new StickController(PRIMARY_CONTROLLER_PORT);
   private final XboxController m_operator = new XboxController(SECONDARY_CONTROLLER_PORT);

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
    }

    public Command getAutonomousCommand() {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("TurnPath", DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 5);
        PIDController x_control = new PIDController(0.2, 0, 0);
        PIDController y_control = new PIDController(0.2, 0, 0);
        ProfiledPIDController angle_control = new ProfiledPIDController(15, 0, 0, new TrapezoidProfile.Constraints(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 5));
        return new PPSwerveControllerCommand(examplePath,() -> m_drivetrainSubsystem.getPose(),m_drivetrainSubsystem.getKinematics(),x_control,y_control, angle_control, (SwerveModuleState[] states) -> m_drivetrainSubsystem.setModuleStates(states),m_drivetrainSubsystem);
    }
}
