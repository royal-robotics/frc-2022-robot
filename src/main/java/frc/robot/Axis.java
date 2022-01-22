package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.Joystick;

public final class Axis {
	public static final double DEADBAND = 0.05;

	private boolean inverted = false;
	private double scale = 1.0;

	private final Joystick joystick;
	private final int axis;

	public Axis(Joystick joystick, int axis) {
		this.joystick = joystick;
		this.axis = axis;
	}

	public boolean isInverted() {
		return inverted;
	}

	public void setInverted(boolean inverted) {
		this.inverted = inverted;
	}

	public double getScale() {
		return scale;
	}

	public void setScale(double scale) {
		this.scale = scale;
	}

	public double getRaw(){
		return joystick.getRawAxis(axis);
	};

	public double get() {
		return get(false, false);
	}

	public double get(boolean squared) {
		return get(squared, false);
	}

	public double get(boolean squared, boolean ignoreScale) {
		double value = getRaw();

		// Invert if axis is inverted
		if (inverted) {
			value = -value;
		}

		// Deadband value
		value = deadband(value, DEADBAND);

		// Square value
		if (squared) {
			value = Math.copySign(value * value, value);
		}

		// Scale value
		if (!ignoreScale) {
			value *= scale;
		}

		return value;
	}

	public Button getButton(double tolerance) {
		return new Button() {
			@Override
			public boolean get() {
				return Math.abs(Axis.this.get()) > tolerance;
			}
		};
	}
    public static double deadband(double input) {
		return deadband(input, 0.025);
	}

	public static double deadband(double input, double buffer) {
		if (Math.abs(input) < buffer) return 0;
		return input;
	}
}
