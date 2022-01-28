// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.input.XboxController;
import edu.wpi.first.wpilibj2.command.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private static final int PRIMARY_CONTROLLER_PORT = 0;

    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    private final XboxController m_controller = new XboxController(PRIMARY_CONTROLLER_PORT);

    public RobotContainer() {
        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem, m_controller));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        m_controller.getA().whenPressed(
            () ->  m_drivetrainSubsystem.zeroGyroscope()
        );
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
