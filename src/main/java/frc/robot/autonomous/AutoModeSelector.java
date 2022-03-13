package frc.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotContainer;
import frc.robot.autonomous.modes.DoNothingAutoMode;
import frc.robot.autonomous.modes.HangarBallTwoBall;
import frc.robot.autonomous.modes.ShootThenBackup;
import frc.robot.autonomous.modes.TestAuto;
import frc.robot.autonomous.modes.ThreeBallAuto;
import frc.robot.autonomous.modes.WallBallTwoBall;

public class AutoModeSelector {
    private final SendableChooser<AutoModeBase> _chooser;

    public AutoModeSelector(RobotContainer robotContainer) {
        _chooser = new SendableChooser<AutoModeBase>();

        this.addAutoMode(new DoNothingAutoMode());
        this.addAutoMode(new ShootThenBackup(robotContainer), /*default*/ true);
        this.addAutoMode(new ThreeBallAuto(robotContainer));
        this.addAutoMode(new TestAuto(robotContainer));
        this.addAutoMode(new WallBallTwoBall(robotContainer));
        this.addAutoMode(new HangarBallTwoBall(robotContainer));

        Shuffleboard.getTab("Competition").add(_chooser).withPosition(0, 0).withSize(2, 1);
    }

    private void addAutoMode(AutoModeBase autoMode) {
        this.addAutoMode(autoMode, false);
    }

    private void addAutoMode(AutoModeBase autoMode, boolean isDefault) {
        if (isDefault) {
            _chooser.setDefaultOption(autoMode.getName(), autoMode);
        } else {
            _chooser.addOption(autoMode.getName(), autoMode);
        }
    }

    public AutoModeBase getAutoMode() {
        return _chooser.getSelected();
    }
}