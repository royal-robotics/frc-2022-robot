package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.Joystick;

public final class Axis {
	public static final double DEADBAND = 0.1;

	private boolean inverted = false;
	private double scale = 1.0;

	private final Joystick joystick;
	private final int axis;

	public Axis(Joystick joystick, int axis, boolean inverted) {
		this.joystick = joystick;
		this.axis = axis;
		this.inverted = inverted;
	}

	public Axis(Joystick joystick, int axis) {
		this(joystick, axis, false);
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
		return get(DEADBAND, false, false);
	}

	public double get(double db) {
		return get(db, false, false);
	}

	public double get(boolean squared) {
		return get(DEADBAND, squared, false);
	}

	public double get(double db, boolean squared, boolean ignoreScale) {
		double value = getRaw();

		// Invert if axis is inverted
		if (inverted) {
			value = -value;
		}

		// Deadband value
		value = deadband(value, db);
		if (value != 0 && db > DEADBAND) {
			double diff = (1 - DEADBAND) / (1 - db);
			if (value > 0) {
				value = diff * value - (diff - 1);
			} else {
				value = diff * value + (diff - 1);
			}
		}

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
		return deadband(input, DEADBAND);
	}

	public static double deadband(double input, double buffer) {
		if (Math.abs(input) < buffer) return 0;
		return input;
	}
}
