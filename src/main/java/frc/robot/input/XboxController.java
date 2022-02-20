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

    private final Axis LeftX;
    private final Axis LeftY;
    private final Axis RightX;
    private final Axis RightY;

    public XboxController(int port) {
        joystick = new Joystick(port);

        A = new JoystickButton(joystick, 1);
        B = new JoystickButton(joystick, 2);
        X = new JoystickButton(joystick, 3);
        Y = new JoystickButton(joystick, 4);

        LeftX = new Axis(joystick, 0, true);
        LeftY = new Axis(joystick, 1, true);
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
}