package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Sensors.Limelight;
import frc.robot.input.StickController;
import frc.robot.input.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootCommand extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final XboxController m_operator;
    private final StickController m_driver;
    //private final XboxController m_driver;
    private final Limelight m_limelight = new Limelight();

    private final double TY_CLOSE = 16;
    private final double TY_FAR = -2.4;
    private final double RPM_CLOSE = 2700;
    private final double RPM_FAR = 4000;

    /*private final double scale = (RPM_FAR - RPM_CLOSE) / (TY_FAR - TY_CLOSE);
    private final double offset = -scale * TY_FAR + RPM_FAR;
    */
    private final double scale = -82.6747;//-50.6693;
    private final double offset = 3989;//3596.1;
    private final double m_slowScale = 0.25;
    private double sin = 0;
    private double cos = 0;
    private double turnOffset = 0;
    private double rpmOffset = 0;

    public ShootCommand(ShooterSubsystem shootSystem, DrivetrainSubsystem driveSystem, XboxController operator, StickController driver) {
    //public ShootCommand(ShooterSubsystem shootSystem, DrivetrainSubsystem driveSystem, XboxController operator, XboxController driver) {
        m_shooterSubsystem = shootSystem;
        m_drivetrainSubsystem = driveSystem;
        m_operator = operator;
        m_driver = driver;
        addRequirements(shootSystem, driveSystem);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.setAngleSetpoint(20);
    }

    @Override
    public void execute() {
        m_limelight.setPipeline(0);
        if (m_limelight.onTarget() && m_shooterSubsystem.getAngle() < 50) {

            var tx = m_limelight.targetX();
            var ty = m_limelight.targetY();
            if (ty > 16 && m_shooterSubsystem.getAngle() > 22.5) {
                m_shooterSubsystem.setAngleSetpoint(20);
            } else if (ty < 0 && m_shooterSubsystem.getAngle() < 22.5) {
                m_shooterSubsystem.setAngleSetpoint(25);
            }

            rpmOffset = (-m_driver.getStrafeAxis().get() * sin * 200) + (m_driver.getForwardAxis().get() * cos * 200); //250 is old value
            //rpmOffset = (m_driver.getLeftX().get() * sin * 200) + (-m_driver.getLeftY().get() * cos * 200);
            double rpm = ty * scale + offset;
            if (rpm < 2900) {
                rpm = 2900;
            }
            rpm += rpmOffset;

            cos = m_drivetrainSubsystem.getGyroscopeRotation().getCos();
            sin = m_drivetrainSubsystem.getGyroscopeRotation().getSin();
            turnOffset = (m_driver.getStrafeAxis().get() * cos * 2) + (-m_driver.getForwardAxis().get() * sin * 2);
            //turnOffset = (-m_driver.getLeftY().get() * cos * 2) + (m_driver.getLeftX().get() * sin * 2);

            ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(-m_driver.getForwardAxis().get(), -m_driver.getStrafeAxis().get(), -tx * 0.18 + turnOffset, m_drivetrainSubsystem.getGyroscopeRotation());
            //ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(m_driver.getLeftY().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, m_driver.getLeftX().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, -tx * 0.18 + turnOffset, m_drivetrainSubsystem.getGyroscopeRotation());
            if(m_driver.getTrigger().getAsBoolean()){
            //if(m_driver.getRightTrigger().get() > 0.5){
                speed = ChassisSpeeds.fromFieldRelativeSpeeds(-m_driver.getForwardAxis().get() * m_slowScale, -m_driver.getStrafeAxis().get() * m_slowScale, -tx * 0.18 * m_slowScale + turnOffset, m_drivetrainSubsystem.getGyroscopeRotation()); //0.18 is an abritary value. It's a value derived from testing the robot.
                //speed = ChassisSpeeds.fromFieldRelativeSpeeds(m_driver.getLeftY().get() * m_slowScale * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, m_driver.getLeftX().get() * m_slowScale * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, -tx * 0.18 * m_slowScale + turnOffset, m_drivetrainSubsystem.getGyroscopeRotation()); //0.18 is an abritary value. It's a value derived from testing the robot.
            }

            m_drivetrainSubsystem.drive(speed);
            m_shooterSubsystem.setSpeedSetpoint(rpm);

            if (m_operator.getLeftBumper().get()) {
                m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kReverse);
            } else {
                m_shooterSubsystem.setSolenoidStates(DoubleSolenoid.Value.kForward, DoubleSolenoid.Value.kForward);
            }
        } else {
            ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(-m_driver.getForwardAxis().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, -m_driver.getStrafeAxis().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, -m_driver.getRotateAxis().get() * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, m_drivetrainSubsystem.getGyroscopeRotation());
            //ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(m_driver.getLeftY().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, m_driver.getLeftX().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, m_driver.getRightX().get() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, m_drivetrainSubsystem.getGyroscopeRotation());
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
