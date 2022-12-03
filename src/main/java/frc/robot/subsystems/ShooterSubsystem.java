package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sensors.Limelight;

import java.util.Map;

public class ShooterSubsystem extends SubsystemBase{

    public final int SHOOTER_ANGLE_MOTOR = 3;
    public final int RIGHT_SHOOTER_WHEEL_MOTOR = 9;
    public final int LEFT_SHOOTER_WHEEL_MOTOR = 8;
    public final int INTAKE_MOTOR = 10;

    public final int EXTEND_INTAKE_LEFT = 1;
    public final int EXTEND_INTAKE_RIGHT = 6;
    public final int KICKER_LEFT = 3;
    public final int KICKER_RIGHT = 4;

    public final double TOP_ANGLE = -20;
    public final double BOTTOM_ANGLE = 121;
    public double topVoltage = 2.7941;
    public double bottomVoltage = 2.6013;
    public final double VOLTAGE_RANGE = topVoltage - bottomVoltage;

    private double scale = (BOTTOM_ANGLE - TOP_ANGLE) / (topVoltage - bottomVoltage);
    private double offset = scale * topVoltage + TOP_ANGLE;

    public final double RPM_TOP = 5600;

    private final CANSparkMax m_shooterAngle;
    private final TalonSRX m_rightShooterWheel;
    private final TalonSRX m_leftShooterWheel;
    private final TalonSRX m_intake;

    private final DoubleSolenoid m_extendIntake;
    private final DoubleSolenoid m_kicker;

    private double m_intakeState = 0;
    private DoubleSolenoid.Value m_extendIntakeState = DoubleSolenoid.Value.kForward;
    private DoubleSolenoid.Value m_kickerState = DoubleSolenoid.Value.kForward;

    public NetworkTableEntry m_speedEntry;
    private NetworkTableEntry m_angleEntry;
    private NetworkTableEntry m_enableAngleEntry;

    private AnalogInput m_analogPotentiometer;
    private Encoder m_encoder;

    private double m_angleOutput;
    private double m_angleSetpoint = -20;
    private double m_speedFFOutput;
    private double m_speedOutput;
    private double m_speedSetpoint;

    private PIDController m_angleController;
    private SimpleMotorFeedforward m_speedFeedForward;
    private PIDController m_speedController;

    private DigitalInput m_bottomLimit;
    private DigitalInput m_topLimit;

    private final Limelight m_limelight = new Limelight();
    private boolean m_readyToFire = false;
    private boolean bottomLimitSet = false;
    private boolean topLimitSet = false;

    public ShooterSubsystem(){
        m_shooterAngle = new CANSparkMax(SHOOTER_ANGLE_MOTOR, MotorType.kBrushless);
        m_rightShooterWheel = new TalonSRX(RIGHT_SHOOTER_WHEEL_MOTOR);
        m_leftShooterWheel = new TalonSRX(LEFT_SHOOTER_WHEEL_MOTOR);
        m_intake = new TalonSRX(INTAKE_MOTOR);

        m_extendIntake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, EXTEND_INTAKE_LEFT, EXTEND_INTAKE_RIGHT);
        m_kicker = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KICKER_LEFT, KICKER_RIGHT);

        m_angleController = new PIDController(0.015, 0, 0);
        m_speedFeedForward = new SimpleMotorFeedforward(0.03480, 0.0001996);
        m_speedController = new PIDController(0.001, 0, 0);

        m_angleController.setTolerance(3);
        m_speedController.setTolerance(100);

        m_bottomLimit = new DigitalInput(0);
        m_topLimit = new DigitalInput(1);

        m_limelight.setPipeline(1);

        m_speedEntry = Shuffleboard.getTab("Competition")
            .add("Wheel Speed", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", RPM_TOP, "block increment", 50))
            .withPosition(2, 0)
            .withSize(2, 2)
            .getEntry();

        m_speedEntry.setDouble(3000);

        /*
        m_enableSpeedEntry = Shuffleboard.getTab("Shooter")
            .add("Enable Wheel Speed", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(2, 2)
            .withSize(2, 1)
            .getEntry();
        */

        m_angleEntry = Shuffleboard.getTab("Shooter")
            .add("Wheel Angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", TOP_ANGLE, "max", BOTTOM_ANGLE, "block increment", 1))
            .withPosition(0, 0)
            .withSize(2, 2)
            .getEntry();

        m_enableAngleEntry = Shuffleboard.getTab("Shooter")
            .add("Enable Wheel Angle", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();

        m_analogPotentiometer = new AnalogInput(0);
        m_encoder = new Encoder(10, 11, true);
        m_encoder.setDistancePerPulse(0.00390625);

        Shuffleboard.getTab("Competition")
        .addNumber("Shooter Angle", () -> getAngle())
        .withPosition(2, 2);

        Shuffleboard.getTab("Competition")
        .addBoolean("Fire Ready", () -> m_readyToFire)
        .withPosition(2, 4);

        Shuffleboard.getTab("Shooter")
        .addNumber("Wheel Angle Sensor Raw", () -> m_analogPotentiometer.getAverageVoltage())
        .withPosition(4, 1);

        Shuffleboard.getTab("Shooter")
        .addNumber("Angle Output", () -> m_angleOutput)
        .withPosition(0, 3);

        Shuffleboard.getTab("Shooter")
        .addNumber("Angle Setpoint", () -> m_angleSetpoint)
        .withPosition(1, 3);

        Shuffleboard.getTab("Shooter")
        .addNumber("Speed FF Output", () -> m_speedFFOutput)
        .withPosition(2, 3);

        Shuffleboard.getTab("Shooter")
        .addNumber("Speed FB Output", () -> m_speedOutput)
        .withPosition(3, 3);

        Shuffleboard.getTab("Shooter")
        .addNumber("Shooter Output", () -> m_speedOutput + m_speedFFOutput)
        .withPosition(2, 3);

        Shuffleboard.getTab("Shooter")
        .addNumber("Speed Setpoint", () -> m_speedSetpoint)
        .withPosition(5, 3);

        Shuffleboard.getTab("Competition")
        .addNumber("Shooter RPM", () -> m_encoder.getRate() * 60)
        .withPosition(2, 3);

        Shuffleboard.getTab("Shooter")
        .addBoolean("Bottom Limit", () -> m_bottomLimit.get())
        .withPosition(4, 3);

        Shuffleboard.getTab("Shooter")
        .addBoolean("Top Limit", () -> m_topLimit.get())
        .withPosition(4, 4);

        Shuffleboard.getTab("Camera")
        .addBoolean("tv", () ->  m_limelight.onTarget())
        .withPosition(0, 0);

        Shuffleboard.getTab("Camera")
        .addNumber("tx", () -> m_limelight.targetX())
        .withPosition(1, 0);

        Shuffleboard.getTab("Competition")
        .addNumber("Camera ty", () -> m_limelight.targetY())
        .withPosition(0, 3);

        Shuffleboard.getTab("Camera")
        .addNumber("ta", () -> m_limelight.targetArea())
        .withPosition(3, 0);
    }

    public void setMotorStates(double shooterSetpoint, double intake){
        setSpeedSetpoint(shooterSetpoint);
        m_intakeState = intake;
    }

    public void setSolenoidStates(DoubleSolenoid.Value extendIntake, DoubleSolenoid.Value kicker){
        m_extendIntakeState = extendIntake;
        m_kickerState = kicker;
    }

    public double getAngleSetpoint() {
        return m_angleSetpoint;
    }

    public void setAngleSetpoint(double setpoint) {
        if (setpoint < TOP_ANGLE) {
            setpoint = TOP_ANGLE;
        } else if (setpoint > BOTTOM_ANGLE) {
            setpoint = BOTTOM_ANGLE;
        }
        m_angleSetpoint = setpoint;
    }

    public boolean atAngleSetpoint() {
        return m_angleController.atSetpoint();
    }

    public void setAngleSetpointCurrent(){
        setAngleSetpoint(getAngle());
    }

    public double getAngle() {
        return m_analogPotentiometer.getAverageVoltage() * -scale + offset;
    }

    public double getSpeedSetpoint() {
        return m_speedSetpoint;
    }

    public void setSpeedSetpoint(double setpoint) {
        if (setpoint < -RPM_TOP) {
            m_speedSetpoint = -RPM_TOP;
        } else if (setpoint > RPM_TOP) {
            m_speedSetpoint = RPM_TOP;
        } else {
            m_speedSetpoint = setpoint;
        }
    }

    public boolean atSpeedSetpoint() {
        return m_speedController.atSetpoint();
    }

    @Override
    public void periodic(){
        double speedSetpoint = m_speedSetpoint;
        /*
        if (m_enableSpeedEntry.getBoolean(false)) {
            speedSetpoint = m_speedEntry.getDouble(0);
        }
        */

        double speedFF = m_speedFeedForward.calculate(speedSetpoint);
        m_speedFFOutput = speedFF;
        if (m_speedController.getSetpoint() != speedSetpoint)
        {
            m_speedController.setSetpoint(speedSetpoint);
        }

        double speedOutput = m_speedController.calculate(m_encoder.getRate() * 60);
        m_speedOutput = speedOutput;
        double wheelSpeed = speedOutput + speedFF;

        if(speedSetpoint == 0){
            wheelSpeed = 0;
        }

        m_rightShooterWheel.set(ControlMode.PercentOutput, -wheelSpeed);
        m_leftShooterWheel.set(ControlMode.PercentOutput, wheelSpeed);

        m_intake.set(ControlMode.PercentOutput, m_intakeState);

        m_extendIntake.set(m_extendIntakeState);
        m_kicker.set(m_kickerState);

        double angleSetpoint = m_angleSetpoint;
        if (m_enableAngleEntry.getBoolean(false)) {
            angleSetpoint = m_angleEntry.getDouble(0);
        }

        if (m_angleController.getSetpoint() != angleSetpoint)
        {
            m_angleController.setSetpoint(angleSetpoint);
        }

        var shooterAngle = getAngle();
        m_angleOutput = m_angleController.calculate(shooterAngle);
        if (angleSetpoint == BOTTOM_ANGLE && shooterAngle < 117) {
            m_angleOutput = 1.0;
        } else if (angleSetpoint < 40 && shooterAngle > 40) {
            m_angleOutput = -1.0;
        } else if (angleSetpoint == TOP_ANGLE && shooterAngle > -16) {
            m_angleOutput = -1.0;
        }
        m_shooterAngle.set(m_angleOutput);

        m_readyToFire = isReadyToFire();

        if (m_bottomLimit.get() && !bottomLimitSet) {
            bottomVoltage = m_analogPotentiometer.getAverageVoltage();
            topVoltage = bottomVoltage + VOLTAGE_RANGE;

            scale = (BOTTOM_ANGLE - TOP_ANGLE) / (topVoltage - bottomVoltage);
            offset = scale * topVoltage + TOP_ANGLE;
            bottomLimitSet = true;
        } else {
            bottomLimitSet = false;
        }

        if(m_topLimit.get() && !topLimitSet) {
            topVoltage = m_analogPotentiometer.getAverageVoltage();
            bottomVoltage = topVoltage - VOLTAGE_RANGE;

            scale = (BOTTOM_ANGLE - TOP_ANGLE) / (topVoltage - bottomVoltage);
            offset = scale * topVoltage + TOP_ANGLE;
            topLimitSet = true;
        } else {
            topLimitSet = false;
        }
    }

    private boolean isReadyToFire() {
        var tx = m_limelight.targetX();
        return m_limelight.onTarget() && (tx > -1.5 && tx < 1.5) && m_angleController.atSetpoint() && m_speedController.atSetpoint();
    }
}
