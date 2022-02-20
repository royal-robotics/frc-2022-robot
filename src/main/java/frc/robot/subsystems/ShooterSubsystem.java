package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

    public final int SHOOTER_ANGLE_MOTOR = 3;
    public final int RIGHT_SHOOTER_WHEEL_MOTOR = 8;
    public final int LEFT_SHOOTER_WHEEL_MOTOR = 9;
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
    private DoubleSolenoid.Value m_extendIntakeState = DoubleSolenoid.Value.kOff;
    private DoubleSolenoid.Value m_kickerState = DoubleSolenoid.Value.kOff;

    public ShooterSubsystem(){
        m_shooterAngle = new CANSparkMax(SHOOTER_ANGLE_MOTOR, MotorType.kBrushless);
        m_rightShooterWheel = new TalonSRX(RIGHT_SHOOTER_WHEEL_MOTOR);
        m_leftShooterWheel = new TalonSRX(LEFT_SHOOTER_WHEEL_MOTOR);
        m_intake = new TalonSRX(INTAKE_MOTOR);

        m_extendIntake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, EXTEND_INTAKE_LEFT, EXTEND_INTAKE_RIGHT);
        m_kicker = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KICKER_LEFT, KICKER_RIGHT);
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

    @Override
    public void periodic(){
        m_shooterAngle.set(m_shooterAngleState);
        m_rightShooterWheel.set(ControlMode.PercentOutput, m_shooterWheelState);
        m_leftShooterWheel.set(ControlMode.PercentOutput, m_shooterWheelState);
        m_intake.set(ControlMode.PercentOutput, m_intakeState);

        m_extendIntake.set(m_extendIntakeState);
        m_kicker.set(m_kickerState);
    }
}
