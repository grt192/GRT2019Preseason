package frc.fieldmapping;

import frc.swerve.SwerveData;

public abstract class PositionTracker {

	protected final double dt;

	public PositionTracker(double dt) {
		this.dt = dt;
		reset();
	}

	public abstract void update(SwerveData data);

	public void reset() {
		set(0, 0);
	}

	public abstract void set(double x, double y);

	public abstract double getX();

	public abstract double getY();
}
