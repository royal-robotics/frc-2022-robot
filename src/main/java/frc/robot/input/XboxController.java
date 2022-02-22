package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public final class XboxController {
    private final Joystick joystick;
    private final Button A;
    private final Button B;
    private final Button Y;
    private final Button X;
    private final Button LeftBumper;
    private final Button RightBumper;

    private final Axis LeftX;
    private final Axis LeftY;
    private final Axis RightX;
    private final Axis RightY;
    private final Axis LeftTrigger;

    public XboxController(int port) {
        joystick = new Joystick(port);

        A = new JoystickButton(joystick, 1);
        B = new JoystickButton(joystick, 2);
        X = new JoystickButton(joystick, 3);
        Y = new JoystickButton(joystick, 4);
        LeftBumper = new JoystickButton(joystick, 5);
        RightBumper = new JoystickButton(joystick, 6);

        LeftX = new Axis(joystick, 0, true);
        LeftY = new Axis(joystick, 1, true);
        LeftTrigger = new Axis(joystick, 2, true);
        RightX = new Axis(joystick, 4, true);
        RightY = new Axis(joystick, 5, true);
    }

    public Axis getLeftX() {
        return LeftX;
    }

    public Axis getLeftY() {
        return LeftY;
    }

    public Axis getRightX() {
        return RightX;
    }

    public Axis getRightY() {
        return RightY;
    }

    public Axis getLeftTrigger() {
        return LeftTrigger;
    }

    public Button getA() {
        return A;
    }

    public Button getB() {
        return B;
    }

    public Button getX() {
        return X;
    }

    public Button getY() {
        return Y;
    }

    public Button getLeftBumper() {
        return LeftBumper;
    }

    public Button getRightBumper() {
        return RightBumper;
    }
}