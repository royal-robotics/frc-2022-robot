package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public abstract class DriveCommandBase extends CommandBase {
    private final DrivetrainSubsystem m_subsystem;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier m_slowSupplier;

    public DriveCommandBase(
            DrivetrainSubsystem subsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier slowSupplier) {
        m_subsystem = subsystem;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_rotationSupplier = rotationSupplier;
        m_slowSupplier = slowSupplier;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (DriverStation.isTeleop()) {
            if (isNoInput()) {
                m_subsystem.setStableModuleStates();
            } else {
                m_subsystem.drive(getInputChassisSpeeds());
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean isNoInput() {
        return m_rotationSupplier.getAsDouble() == 0 &&
            m_translationXSupplier.getAsDouble() == 0 &&
            m_translationYSupplier.getAsDouble() == 0;
    }
    private ChassisSpeeds getInputChassisSpeeds() {
        if (m_slowSupplier.getAsBoolean()) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                m_translationXSupplier.getAsDouble() * 0.25,
                m_translationYSupplier.getAsDouble() * 0.25,
                m_rotationSupplier.getAsDouble() * 0.25,
                m_subsystem.getGyroscopeRotation());
        } else {
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(),
                m_rotationSupplier.getAsDouble(),
                m_subsystem.getGyroscopeRotation());
        }
    }
}
