package frc.robot;

public class Constants {

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6604;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4191;

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

    public static final int DRIVETRAIN_PIGEON_ID = 0;

    // In degrees
    //public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = Math.toRadians(65.89495849609318 - 52.944824218748884);
    //public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = Math.toRadians(340.0961303710937 - 297.8811645507811);
    //public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = Math.toRadians(219.12030029296875 - 201.41857910156259);
    //public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = Math.toRadians(30 + 183.38239746093726 - 153.22492675781223);

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(287.3);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(192.0);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(63.6);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(149.1);

    public static final int PRIMARY_CONTROLLER_PORT = 0;
}
