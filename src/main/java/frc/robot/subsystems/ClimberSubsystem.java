package frc.robot.subsystems;

//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    public final int S1 = 1;
    public final int S2 = 2;
    public final int S4 = 4;

    //private final CANSparkMax m_S1;
    //private final CANSparkMax m_S2;
    //private final CANSparkMax m_S4;

    private double[] m_CANState = new double[3];

    public ClimberSubsystem(){
       // m_S1 = new CANSparkMax(S1, CANSparkMax.MotorType.kBrushless);
       // m_S2 = new CANSparkMax(S2, CANSparkMax.MotorType.kBrushless);
       // m_S4 = new CANSparkMax(S4, CANSparkMax.MotorType.kBrushless);

        m_CANState[0] = 0;
        m_CANState[1] = 0;
        m_CANState[2] = 0;
    }

    public void setModuleStates(double[] states){
        m_CANState = states;
    }

    @Override
    public void periodic() {
        double[] states = m_CANState;

       // m_S1.set(states[0]);
       // m_S2.set(states[1]);
       // m_S4.set(states[2]);
    }
}
