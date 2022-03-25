package frc.robot.subsystems;


import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;



public class ClimberSubsystem extends SubsystemBase{

    public final int LEFT_CLIMBER_MOTOR = 1;
    public final int RIGHT_CLIMBER_MOTOR = 2;
    public final int CLIMBER_ANGLE_MOTOR = 4;


    private final CANSparkMax m_leftClimber;
    private final CANSparkMax m_rightClimber;
    private final CANSparkMax m_climberAngle;

    private double m_climberState = 0;
    private double m_angleSetpoint = 0;
    private double m_distanceSetpoint = 0;

    public final double TOP_ANGLE = 90;
    public final double BOTTOM_ANGLE = -23;
    public final double TOP_VOLTAGE = 2.574;
    public final double BOTTOM_VOLTAGE = 2.022;

    public final double TOP_DISTANCE = 32;

    private double scale = (BOTTOM_ANGLE - TOP_ANGLE) / (TOP_VOLTAGE - BOTTOM_VOLTAGE);
    private double offset = scale * TOP_VOLTAGE + TOP_ANGLE;

    private AnalogInput m_analogPotentiometer;
    private Encoder m_encoder;

    private PIDController m_angleController;
    private PIDController m_distanceController;

    private NetworkTableEntry m_distanceEntry;
    private NetworkTableEntry m_enableDistanceEntry;
    private NetworkTableEntry m_angleEntry;
    private NetworkTableEntry m_enableAngleEntry;

    private double m_angleOutput;
    private double m_distanceOutput;

    public ClimberSubsystem(){
        m_leftClimber = new CANSparkMax(LEFT_CLIMBER_MOTOR, MotorType.kBrushless);
        m_rightClimber = new CANSparkMax(RIGHT_CLIMBER_MOTOR, MotorType.kBrushless);
        m_climberAngle = new CANSparkMax(CLIMBER_ANGLE_MOTOR, MotorType.kBrushless);

        m_analogPotentiometer = new AnalogInput(1);
        m_encoder = new Encoder(12, 13, false);
        m_encoder.setDistancePerPulse(0.02360515021);

        m_angleController = new PIDController(0.05, 0, 0);
        m_distanceController = new PIDController(0.5, 0, 0);

        Shuffleboard.getTab("Competition")
        .addNumber("Climber Angle", () -> m_analogPotentiometer.getAverageVoltage() * -scale + offset)
        .withPosition(3, 2);
        Shuffleboard.getTab("Climber")
        .addNumber("Potentiometer Raw", () -> m_analogPotentiometer.getAverageVoltage())
        .withPosition(5, 1);
        Shuffleboard.getTab("Competition")
        .addNumber("Climber Height", () -> m_encoder.getDistance())
        .withPosition(3, 3);

        m_distanceEntry = Shuffleboard.getTab("Climber")
            .add("Climber Distance", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", TOP_DISTANCE, "block increment", 1))
            .withPosition(2, 0)
            .withSize(2, 2)
            .getEntry();

        m_enableDistanceEntry = Shuffleboard.getTab("Climber")
            .add("Enable Climber Distance", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(2, 2)
            .withSize(2, 1)
            .getEntry();

        m_angleEntry = Shuffleboard.getTab("Climber")
            .add("Climber Angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", BOTTOM_ANGLE, "max", TOP_ANGLE, "block increment", 1))
            .withPosition(0, 0)
            .withSize(2, 2)
            .getEntry();

        m_enableAngleEntry = Shuffleboard.getTab("Climber")
            .add("Enable Climber Angle", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();

        Shuffleboard.getTab("Climber")
            .addNumber("Angle Output", () -> m_angleOutput)
            .withPosition(4, 0);

        Shuffleboard.getTab("Climber")
            .addNumber("Distance Output", () -> m_distanceOutput)
            .withPosition(4, 1);

        Shuffleboard.getTab("Climber")
        .addNumber("Angle Setpoint", () -> m_angleSetpoint)
        .withPosition(2,3);

        Shuffleboard.getTab("Climber")
        .addNumber("Distance Setpoint", () -> m_distanceSetpoint)
        .withPosition(3,3);

        Shuffleboard.getTab("Climber")
        .addNumber("Distance Power", () -> m_climberState)
        .withPosition(4,3);
    }

    public void setMotorStates(double climberState, double climberAngleState){
        m_climberState = climberState;
    }

    public void resetEncoder(){
        setDistanceSetpoint(0);
        m_encoder.reset();
    }

    public void setAngleSetpoint(double setpoint) {
       if (setpoint < BOTTOM_ANGLE) {
            setpoint = BOTTOM_ANGLE;
        } else if (setpoint > TOP_ANGLE) {
            setpoint = TOP_ANGLE;
        }

        m_angleSetpoint = setpoint;
    }

    public void setAngleSetpointCurrent(){
        setAngleSetpoint(getAngle());
    }
    
    public void setDistanceSetpoint(double setpoint) {
         if (setpoint < 0) {
             setpoint = 0;
         } else if (setpoint > TOP_DISTANCE) {
             setpoint = TOP_DISTANCE;
         }

         m_distanceSetpoint = setpoint;
     }

     public double getAngleSetpoint(){
        return m_angleSetpoint;
     }

     public double getAngle() {
         return m_analogPotentiometer.getAverageVoltage() * -scale + offset;
     }

     public double getDistanceSetpoint() {
         return m_distanceSetpoint;
     }

     public double getDistance() {
        return m_encoder.getDistance();
     }

    @Override
    public void periodic() {
        //double climbSpeed = m_climberState;
        //m_leftClimber.set(climbSpeed);
        //m_rightClimber.set(-climbSpeed);

        //m_climberAngle.set(m_climberAngleState);

        double angleSetpoint = m_angleSetpoint;
        if (m_enableAngleEntry.getBoolean(false)) {
            angleSetpoint = m_angleEntry.getDouble(0);
        }

        if (m_angleController.getSetpoint() != angleSetpoint)
        {
            m_angleController.setSetpoint(angleSetpoint);
        }
        m_angleOutput = m_angleController.calculate(m_analogPotentiometer.getAverageVoltage() * -scale + offset);
        m_climberAngle.set(m_angleOutput);

        double distanceSetpoint = m_distanceSetpoint;
        if (m_enableDistanceEntry.getBoolean(false)) {
            distanceSetpoint = m_distanceEntry.getDouble(0);
        }

        if (m_distanceController.getSetpoint() != distanceSetpoint)
        {
            m_distanceController.setSetpoint(distanceSetpoint);
        }
        m_distanceOutput = m_distanceController.calculate(m_encoder.getDistance());
        m_leftClimber.set(m_distanceOutput);
        m_rightClimber.set(-m_distanceOutput);
    }
}
