// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
//import frc.robot.commands.DefaultDriveCommand;
//import frc.robot.subsystems.DrivetrainSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  //private final DriveCommand m_autoCommand = new DriveCommand(m_DrivetrainSubsystem, );

  private final XboxController m_controller = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    //flightStick.getForwardAxis().setInverted(true);
    //flightStick.getStrafeAxis().setInverted(true);
    //flightStick.getRotateAxis().setInverted(true);

    m_drivetrainSubsystem.setDefaultCommand(
      new DriveCommand(
            m_drivetrainSubsystem,
            ()-> modifyAxis(m_controller.getForwardAxis().get())* DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> -modifyAxis(m_controller.getStrafeAxis().get()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            ()-> -modifyAxis(m_controller.getRotateAxis().get()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    

    //m_controller.getStrafeAxis().setInverted(true);
    //m_controller.getRotateAxis().setInverted(true);

    //CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*new Button(m_controller::getBackButton)
    // No requirements because we don't need to interrupt anything
    .whenPressed(DrivetrainSubsystem::zeroGyroscope);
    */

        m_controller.getA().whenPressed(
            () ->  m_drivetrainSubsystem.zeroGyroscope()
        );
        

        /*
        m_controller.getX.whenPressed(
            DrivetrainSubsystem::resetWheelAngles
        );
        */
        
        /*flightStick.getc().whenPressed(
            () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
        );*/
            
          
       /* 
        flightStick.getRawButton(9).whenPressed(
                () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
        );
        */
        /*
        flightStick.getT2().whenPressed(
                drivetrainSubsystem::resetWheelAngles
        );
        */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An DriveCommand will run in autonomous
    //return m_autoCommand;
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
