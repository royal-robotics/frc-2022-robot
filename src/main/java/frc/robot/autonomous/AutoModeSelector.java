package frc.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotContainer;
import frc.robot.autonomous.modes.DoNothingAutoMode;
import frc.robot.autonomous.modes.ShootThenBackup;

public class AutoModeSelector {
    private final SendableChooser<AutoModeBase> _chooser;

    public AutoModeSelector(RobotContainer robotContainer) {
        _chooser = new SendableChooser<AutoModeBase>();
        var doNothing = new DoNothingAutoMode();
        var shootAndBackup = new ShootThenBackup(robotContainer);

        _chooser.addOption(doNothing.getName(), doNothing);
        _chooser.setDefaultOption(shootAndBackup.getName(), shootAndBackup);

        Shuffleboard.getTab("Competition").add(_chooser);
    }

    public AutoModeBase getAutoMode() {
        return _chooser.getSelected();
    }
}