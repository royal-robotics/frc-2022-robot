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

    private final Axis forwardAxis;
    private final Axis rotateAxis;
    private final Axis strafeAxis;

    public XboxController(int port) {
        joystick = new Joystick(port);

        A = new JoystickButton(joystick, 1);
        B = new JoystickButton(joystick, 2);
        X = new JoystickButton(joystick, 3);
        Y = new JoystickButton(joystick, 4);

        forwardAxis = new Axis(joystick, 1, true);
        rotateAxis = new Axis(joystick, 4, true);
        strafeAxis = new Axis(joystick, 0, true);
    }

    public Axis getForwardAxis() {
        return forwardAxis;
    }

    public Axis getRotateAxis() {
        return rotateAxis;
    }

    public Axis getStrafeAxis() {
        return strafeAxis;
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