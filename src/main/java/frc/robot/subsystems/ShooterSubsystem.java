package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public final double TOP_ANGLE = -21;
    public final double BOTTOM_ANGLE = 134;
    public final double TOP_VOLTAGE = 2.787;
    public final double BOTTOM_VOLTAGE = 2.583;

    private double scale = (BOTTOM_ANGLE - TOP_ANGLE) / (TOP_VOLTAGE - BOTTOM_VOLTAGE);
    private double offset = scale * TOP_VOLTAGE + TOP_ANGLE;

    private final CANSparkMax m_shooterAngle;
    private final TalonSRX m_rightShooterWheel;
    private final TalonSRX m_leftShooterWheel;
    private final TalonSRX m_intake;

    private final DoubleSolenoid m_extendIntake;
    private final DoubleSolenoid m_kicker;

    private double m_shooterWheelState = 0;
    private double m_intakeState = 0;
    private DoubleSolenoid.Value m_extendIntakeState = DoubleSolenoid.Value.kForward;
    private DoubleSolenoid.Value m_kickerState = DoubleSolenoid.Value.kForward;

    private NetworkTableEntry m_speedEntry;
    private NetworkTableEntry m_enableSpeedEntry;
    private NetworkTableEntry m_angleEntry;
    private NetworkTableEntry m_enableAngleEntry;

    private AnalogInput m_analogPotentiometer;
    private Encoder m_encoder;

    private double m_angleOutput;
    private double m_angleSetpoint = 45;
    private double m_speedFFOutput;
    private double m_speedOutput;
    private double m_speedSetpoint;

    private PIDController m_angleController = new PIDController(0.01, 0, 0);
    private SimpleMotorFeedforward m_speedFeedForward = new SimpleMotorFeedforward(0.00002, 0.00026);
    private PIDController m_speedController = new PIDController(0.001, 0, 0);

    public ShooterSubsystem(){
        m_shooterAngle = new CANSparkMax(SHOOTER_ANGLE_MOTOR, MotorType.kBrushless);
        m_rightShooterWheel = new TalonSRX(RIGHT_SHOOTER_WHEEL_MOTOR);
        m_leftShooterWheel = new TalonSRX(LEFT_SHOOTER_WHEEL_MOTOR);
        m_intake = new TalonSRX(INTAKE_MOTOR);

        m_extendIntake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, EXTEND_INTAKE_LEFT, EXTEND_INTAKE_RIGHT);
        m_kicker = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KICKER_LEFT, KICKER_RIGHT);

        m_speedEntry = Shuffleboard.getTab("Control")
            .add("Wheel Speed", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 3800, "block increment", 50))
            .withPosition(2, 0)
            .withSize(2, 2)
            .getEntry();

        m_enableSpeedEntry = Shuffleboard.getTab("Control")
            .add("Enable Wheel Speed", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(2, 2)
            .withSize(2, 1)
            .getEntry();

        m_angleEntry = Shuffleboard.getTab("Control")
            .add("Wheel Angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -20, "max", 130, "block increment", 1))
            .withPosition(0, 0)
            .withSize(2, 2)
            .getEntry();

        m_enableAngleEntry = Shuffleboard.getTab("Control")
            .add("Enable Wheel Angle", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();

        m_analogPotentiometer = new AnalogInput(0);
        m_encoder = new Encoder(10, 11, true);
        m_encoder.setDistancePerPulse(0.00390625);

        Shuffleboard.getTab("Control")
        .addNumber("Wheel Angle Sensor", () -> m_analogPotentiometer.getAverageVoltage() * -scale + offset)
        .withPosition(4, 0);

        Shuffleboard.getTab("Control")
        .addNumber("Wheel Angle Sensor Raw", () -> m_analogPotentiometer.getAverageVoltage())
        .withPosition(4, 1);

        Shuffleboard.getTab("Control")
        .addNumber("Angle Output", () -> m_angleOutput)
        .withPosition(0, 3);

        Shuffleboard.getTab("Control")
        .addNumber("Angle Setpoint", () -> m_angleSetpoint)
        .withPosition(1, 3);

        Shuffleboard.getTab("Control")
        .addNumber("Speed FF Output", () -> m_speedFFOutput)
        .withPosition(2, 3);

        Shuffleboard.getTab("Control")
        .addNumber("Speed FB Output", () -> m_speedOutput)
        .withPosition(3, 3);

        Shuffleboard.getTab("Control")
        .addNumber("Speed Total Output", () -> m_speedOutput + m_speedFFOutput)
        .withPosition(4, 3);

        Shuffleboard.getTab("Control")
        .addNumber("Speed Setpoint", () -> m_speedSetpoint)
        .withPosition(5, 3);

        Shuffleboard.getTab("Control")
        .addNumber("Speed Sensor", () -> m_encoder.getRate() * 60)
        .withPosition(4, 2);
    }

    public void setMotorStates(double shooterWheel, double intake){
        m_shooterWheelState = shooterWheel;
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
        if (setpoint < -20) {
            setpoint = -20;
        } else if (setpoint > 135) {
            setpoint = 135;
        }
        m_angleSetpoint = setpoint;
    }

    public double getSpeedSetpoint() {
        return m_speedSetpoint;
    }

    public void setSpeedSetpoint(double setpoint) {
        if (setpoint < 0) {
            setpoint = 0;
        } else if (setpoint > 3800) {
            setpoint = 3800;
        }
        m_speedSetpoint = setpoint;
    }

    @Override
    public void periodic(){
        double wheelSpeed = m_shooterWheelState;
        double speedSetpoint = m_speedSetpoint;
        if (m_enableSpeedEntry.getBoolean(false)) {
            speedSetpoint = m_speedEntry.getDouble(0);
        }
        double speedFF = m_speedFeedForward.calculate(speedSetpoint);
        m_speedFFOutput = speedFF;
        if (m_speedController.getSetpoint() != speedSetpoint)
        {
            m_speedController.setSetpoint(speedSetpoint);
        }
        double speedOutput = m_speedController.calculate(m_encoder.getRate() * 60);
        m_speedOutput = speedOutput;
        if (m_enableSpeedEntry.getBoolean(false)) {
            wheelSpeed = speedOutput + speedFF;
        }
        m_rightShooterWheel.set(ControlMode.PercentOutput, wheelSpeed);
        m_leftShooterWheel.set(ControlMode.PercentOutput, -wheelSpeed);

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
        m_angleOutput = m_angleController.calculate(m_analogPotentiometer.getAverageVoltage() * -scale + offset);
        m_shooterAngle.set(m_angleOutput);
    }
}
