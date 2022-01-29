package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6604;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4191;

    // Based on the absolute position of the encoders, use this procedure to calculate:
    // https://github.com/SwerveDriveSpecialties/swerve-template#setting-up-module-offsets
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(277.78);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(176.70);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(59.01);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(148.31);

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 0;
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR  = 1;

    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7;

    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 0;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 2;

    // This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
    public static final double MAX_VOLTAGE = 6.0;

    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
    //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
    //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        SdsModuleConfigurations.MK3_FAST.getDriveReduction() *
        SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI;

    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    public SwerveDriveKinematics getKinematics(){
        return m_kinematics;
    }

    // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.
    private final PigeonIMU m_pigeon = new PigeonIMU(0);

    public SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,getGyroscopeRotation());

    public Pose2d m_pose = new Pose2d();
    public Pose2d getPose(){
        return m_pose;
    }

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private final ShuffleboardTab m_dashboardTab = Shuffleboard.getTab("Drivetrain");

    private SwerveModuleState[] m_ModuleState = new SwerveModuleState[4];

    public DrivetrainSubsystem() {
        m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            m_dashboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET);

        m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
            m_dashboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET);

        m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
            m_dashboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET);

        m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
            m_dashboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0),
            Mk3SwerveModuleHelper.GearRatio.FAST,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET);

        m_dashboardTab.addNumber("Gryo", () -> getGyroscopeRotation().getDegrees());
    }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is
   * currently facing to the 'forwards' direction.
   */
    public void zeroGyroscope() {
        m_pigeon.setFusedHeading(0.0);
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        m_ModuleState = states;
    }

    public void setModuleStates(SwerveModuleState[] states){
        m_ModuleState = states;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_ModuleState;
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_pose = m_odometry.update(getGyroscopeRotation(), states);
        
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }
}