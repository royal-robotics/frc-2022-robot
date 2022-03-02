package frc.robot.autonomous;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotContainer;
import frc.robot.autonomous.modes.DoNothingAutoMode;

public class AutoModeSelector {
    private final SendableChooser<AutoModeBase> _chooser;

    public AutoModeSelector(RobotContainer robotContainer) {
        _chooser = new SendableChooser<AutoModeBase>();
        var doNothing = new DoNothingAutoMode();
        _chooser.setDefaultOption(doNothing.getName(), doNothing);
        Shuffleboard.getTab("Competition").add(_chooser);
    }

    public AutoModeBase getAutoMode() {
        return _chooser.getSelected();
    }
}