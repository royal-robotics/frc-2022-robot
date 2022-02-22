package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
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

    private final CANSparkMax m_shooterAngle;
    private final TalonSRX m_rightShooterWheel;
    private final TalonSRX m_leftShooterWheel;
    private final TalonSRX m_intake;

    private final DoubleSolenoid m_extendIntake;
    private final DoubleSolenoid m_kicker;

    private double m_shooterAngleState = 0;
    private double m_shooterWheelState = 0;
    private double m_intakeState = 0;
    private DoubleSolenoid.Value m_extendIntakeState = DoubleSolenoid.Value.kForward;
    private DoubleSolenoid.Value m_kickerState = DoubleSolenoid.Value.kForward;

    private NetworkTableEntry m_speedEntry;

    private AnalogPotentiometer m_analogPotentiometer;
    private Encoder m_encoder;

    private double m_output;
    private double m_setpoint;

    private PIDController m_angleController = new PIDController(0.002, 0, 0);

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
            .withProperties(Map.of("min", 0, "max", 1, "block increment", 0.01))
            .withPosition(2, 0)
            .withSize(2, 2)
            .getEntry();

        m_analogPotentiometer = new AnalogPotentiometer(0, 3000, -1590);
        m_encoder = new Encoder(10, 11, true);
        m_encoder.setDistancePerPulse(0.00390625);

        Shuffleboard.getTab("Control")
        .addNumber("Wheel Angle", () -> m_analogPotentiometer.get())
        .withPosition(4, 0);

        Shuffleboard.getTab("Control")
        .addNumber("PID output", () -> m_output)
        .withPosition(5, 0);

        Shuffleboard.getTab("Control")
        .addNumber("Setpoint", () -> m_setpoint)
        .withPosition(6, 0);

        Shuffleboard.getTab("Control")
        .addNumber("Encoder", () -> m_encoder.getRate())
        .withPosition(4, 2);
    }

    public void setMotorStates(double shooterAngle, double shooterWheel, double intake){
        m_shooterAngleState = shooterAngle;
        m_shooterWheelState = shooterWheel;
        m_intakeState = intake;
    }

    public void setSolenoidStates(DoubleSolenoid.Value extendIntake, DoubleSolenoid.Value kicker){
        m_extendIntakeState = extendIntake;
        m_kickerState = kicker;
    }

    public void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
    }

    @Override
    public void periodic(){
        m_shooterAngle.set(m_shooterAngleState);

        double speedEntry = m_speedEntry.getDouble(0);
        if (speedEntry != 0) {
            m_rightShooterWheel.set(ControlMode.PercentOutput, speedEntry);
            m_leftShooterWheel.set(ControlMode.PercentOutput, -speedEntry);
        } else {
            double wheelSpeed = m_shooterWheelState;
            m_rightShooterWheel.set(ControlMode.PercentOutput, wheelSpeed);
            m_leftShooterWheel.set(ControlMode.PercentOutput, -wheelSpeed);
        }

        m_intake.set(ControlMode.PercentOutput, m_intakeState);

        m_extendIntake.set(m_extendIntakeState);
        m_kicker.set(m_kickerState);

        double setpoint = m_setpoint;
        if (m_angleController.getSetpoint() != setpoint)
        {
            m_angleController.setSetpoint(setpoint);
        }
        m_output = -m_angleController.calculate(m_analogPotentiometer.get());
    }
}
