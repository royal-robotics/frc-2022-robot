package frc.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotContainer;
import frc.robot.autonomous.modes.DoNothingAutoMode;
import frc.robot.autonomous.modes.HangarBallTwoBall;
import frc.robot.autonomous.modes.MiddleBallTwoBall;
import frc.robot.autonomous.modes.ShootThenBackup;
import frc.robot.autonomous.modes.TestAuto;
import frc.robot.autonomous.modes.ThreeBallAuto;
import frc.robot.autonomous.modes.WallBallTwoBall;

import java.util.function.*;

public class AutoModeSelector {
    private final SendableChooser<Supplier<AutoModeBase>> _chooser;

    public AutoModeSelector(RobotContainer robotContainer) {
        _chooser = new SendableChooser<Supplier<AutoModeBase>>();

        this.addAutoMode(() -> new DoNothingAutoMode());
        this.addAutoMode(() -> new ShootThenBackup(robotContainer), true);
        //this.addAutoMode(() -> new ThreeBallAuto(robotContainer));
        // this.addAutoMode(() -> new TestAuto(robotContainer));
        //this.addAutoMode(() -> new WallBallTwoBall(robotContainer));
        //this.addAutoMode(() -> new HangarBallTwoBall(robotContainer));
        this.addAutoMode(() -> new MiddleBallTwoBall(robotContainer));

        Shuffleboard.getTab("Competition").add("Auto Mode", _chooser).withPosition(0, 0).withSize(2, 1);
    }

    private void addAutoMode(Supplier<AutoModeBase> autoModeSupplier) {
        this.addAutoMode(autoModeSupplier, false);
    }

    private void addAutoMode(Supplier<AutoModeBase> autoModeSupplier, boolean isDefault) {
        if (isDefault) {
            _chooser.setDefaultOption(autoModeSupplier.get().getName(), autoModeSupplier);
        } else {
            _chooser.addOption(autoModeSupplier.get().getName(), autoModeSupplier);
        }
    }

    public AutoModeBase getAutoMode() {
        return _chooser.getSelected().get();
    }
}