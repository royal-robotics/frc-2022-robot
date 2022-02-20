package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    public final int LEFT_CLIMBER_MOTOR = 1;
    public final int RIGHT_CLIMBER_MOTOR = 2;
    public final int CLIMBER_ANGLE_MOTOR = 4;

    private final CANSparkMax m_leftClimber;
    private final CANSparkMax m_rightClimber;
    private final CANSparkMax m_climberAngle;

    private double m_climberState = 0;
    private double m_climberAngleState = 0;

    public ClimberSubsystem(){
        m_leftClimber = new CANSparkMax(LEFT_CLIMBER_MOTOR, MotorType.kBrushless);
        m_rightClimber = new CANSparkMax(RIGHT_CLIMBER_MOTOR, MotorType.kBrushless);
        m_climberAngle = new CANSparkMax(CLIMBER_ANGLE_MOTOR, MotorType.kBrushless);
    }

    public void setMotorStates(double climberState, double climberAngleState){
        m_climberState = climberState;
        m_climberAngleState = climberAngleState;
    }

    @Override
    public void periodic() {
        m_leftClimber.set(m_climberState);
        m_rightClimber.set(m_climberState);
        m_climberAngle.set(m_climberAngleState);
    }
}
