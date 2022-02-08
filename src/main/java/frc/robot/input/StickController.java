package frc.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public final class StickController {
    private final Joystick joystick;

    private final Button t1;
    private final Button t2;
    private final Button c;
    private final Button mode;

    private final Axis forwardAxis;
    private final Axis rotateAxis;
    private final Axis strafeAxis;

    public StickController(int port) {
        joystick = new Joystick(port);

        t1 = new JoystickButton(joystick,11);
        t2 = new JoystickButton(joystick, 10);
        c = new JoystickButton(joystick,5);
        mode = new JoystickButton(joystick,24);

        forwardAxis = new Axis(joystick, 1);
        rotateAxis = new Axis(joystick, 2);
        strafeAxis = new Axis(joystick, 0);
    }

    public Axis getForwardAxis(){
        return forwardAxis;
    }

    public Axis getRotateAxis(){
        return rotateAxis;
    }

    public Axis getStrafeAxis(){
        return strafeAxis;
    }

    public Button getT1(){
        return t1;
    }
    public Button getT2(){
        return t2;
    }

    public Button getc(){
        return c;
    }

    public Button getMode(){
        return mode;
    }
}