// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.input.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private static final int PRIMARY_CONTROLLER_PORT = 0;
    private static final int SECONDARY_CONTROLLER_PORT = 1;

    //private final XboxController m_controller = new XboxController(PRIMARY_CONTROLLER_PORT);
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
        m_controller.getT1().whenPressed(
            () -> drivetrainSubsystem.zeroGyroscope()
        );
        /*m_controller.getX().whenPressed(
            () -> drivetrainSubsystem.zeroGyroscope()
        );*/
        m_operator.getA().whenPressed(
            () -> {
                if (m_operator.getLeftTrigger().get() == 0) {
                    m_pickupCommand.schedule();
                }
            }
        );
        m_operator.getA().whenReleased(
            () -> {
                if (m_pickupCommand.isScheduled()) {
                    m_pickupCommand.cancel();
                }
            }
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
}
