package frc.fieldmapping;

import frc.swerve.SwerveData;

public class EncoderPositionTracker {

	private double dt;
	private double x, y;

	public EncoderPositionTracker(double dt) {
		this.dt = dt;
	}

	public void update(SwerveData data) {
		x += data.encoderVX * dt;
		y += data.encoderVY * dt;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public void reset() {
		set(0, 0);
	}

	public void set(double x, double y) {
		this.x = x;
		this.y = y;
	}

}
