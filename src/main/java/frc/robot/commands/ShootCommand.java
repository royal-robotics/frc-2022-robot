package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Sensors.Limelight;
import frc.robot.input.StickController;
import frc.robot.input.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final XboxController m_operator;
    private final StickController m_driver;
    private final Limelight m_limelight = new Limelight();

    private final double TY_CLOSE = 16;
    private final double TY_FAR = -2.4;
    private final double RPM_CLOSE = 2700;
    private final double RPM_FAR = 4000;

    /*private final double scale = (RPM_FAR - RPM_CLOSE) / (TY_FAR - TY_CLOSE);
    private final double offset = -scale * TY_FAR + RPM_FAR;
    */
    private final double scale = -50.6693;
    private final double offset = 3546.1;
    private final double m_slowScale = 0.25;

    public ShootCommand(ShooterSubsystem shootSystem, DrivetrainSubsystem driveSystem, XboxController operator, StickController driver) {
        m_shooterSubsystem = shootSystem;
        m_drivetrainSubsystem = driveSystem;
        m_operator = operator;
        m_driver = driver;
        addRequirements(shootSystem, driveSystem);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.setAngleSetpoint(25);
    }

    @Override
    public void execute() {
        m_limelight.setPipeline(0);
        if (m_limelight.onTarget() && m_shooterSubsystem.getAngle() < 50) {

            var tx = m_limelight.targetX();
            var ty = m_limelight.targetY();
            if (ty > 20 && m_shooterSubsystem.getAngle() > 22.5) {
                m_shooterSubsystem.setAngleSetpoint(20);
            } else if (ty < 12 && m_shooterSubsystem.getAngle() < 22.5) {
                m_shooterSubsystem.setAngleSetpoint(25);
            }

            double rpm = ty * scale + offset;

            ChassisSpeeds speed = new ChassisSpeeds(-m_driver.getForwardAxis().get(), -m_driver.getStrafeAxis().get(), -tx * 0.18);
            if(m_driver.getTrigger().getAsBoolean()){
                speed = new ChassisSpeeds(-m_driver.getForwardAxis().get() * m_slowScale, -m_driver.getStrafeAxis().get() * m_slowScale, -tx * 0.18 * m_slowScale); //0.18 is an abritary value. It's a value derived from testing the robot.
            }

            m_drivetrainSubsystem.drive(speed);
            m_shooterSubsystem.setSpeedSetpoint(rpm);

            if (m_operator.getLeftBumper().get()) {
                m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kReverse);
            } else {
                m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kForward);
            }
        } else {
            ChassisSpeeds speed = new ChassisSpeeds(-m_driver.getForwardAxis().get(), -m_driver.getStrafeAxis().get(), -m_driver.getRotateAxis().get());
            m_drivetrainSubsystem.drive(speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setSpeedSetpoint(0);
        m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kForward);
        m_limelight.setPipeline(1);
    }
}
