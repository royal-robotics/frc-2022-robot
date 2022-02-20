package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

    public final int S3 = 3;
    public final int D8 = 8;
    public final int D9 = 9;
    public final int D10 = 10;

    public final int P1L = 1;
    public final int P1R = 6;
    public final int P2L = 3;
    public final int P2R = 4; 

    private final CANSparkMax m_S3;
    private final TalonSRX m_D8;
    private final TalonSRX m_D9;
    private final TalonSRX m_D10;

    private final DoubleSolenoid m_P1;
    private final DoubleSolenoid m_P2;
    

    private double[] m_MotorState = new double[4];
    private DoubleSolenoid.Value[] m_SolenoidState = new DoubleSolenoid.Value[2];
    


    public ShooterSubsystem(){
       m_S3 = new CANSparkMax(S3, MotorType.kBrushless);
       m_S3.restoreFactoryDefaults(); 
        m_D8 = new TalonSRX(D8);
        m_D9 = new TalonSRX(D9);
        m_D10 = new TalonSRX(D10);

        m_P1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, P1L, P1R);
        m_P2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, P2L, P2R);

        m_P1.set(DoubleSolenoid.Value.kOff);
        m_P2.set(DoubleSolenoid.Value.kOff);

        m_MotorState[0] = 0;
        m_MotorState[1] = 0;
        m_MotorState[2] = 0;
        m_MotorState[3] = 0;

        m_SolenoidState[0] = DoubleSolenoid.Value.kOff;
        m_SolenoidState[1] = DoubleSolenoid.Value.kOff;

    }

    public void setModuleStates(double[] states){
        m_MotorState = states;
    }

    public void setSolenoidStates(DoubleSolenoid.Value[] states){
        m_SolenoidState = states;
    }

    @Override
    public void periodic(){
        double[] states = m_MotorState;
        DoubleSolenoid.Value[] solenoidStates = m_SolenoidState;

        m_S3.set(states[0]);
        m_D8.set(ControlMode.PercentOutput, states[1]);
        m_D9.set(ControlMode.PercentOutput, states[2]);
        m_D10.set(ControlMode.PercentOutput, states[3]);

        m_P1.set(solenoidStates[0]);
        m_P2.set(solenoidStates[1]);   
    }
}
