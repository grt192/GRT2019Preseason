package frc.fieldmapping;

import frc.swerve.SwerveData;

public class EncoderPositionTracker extends PositionTracker {

	private double x, y;

	public EncoderPositionTracker(double dt) {
		super(dt);
	}

	@Override
	public void update(SwerveData data) {
		x += data.encoderVX * dt;
		y += data.encoderVY * dt;
	}

	@Override
	public double getX() {
		return x;
	}

	@Override
	public double getY() {
		return y;
	}

	@Override
	public void set(double x, double y) {
		this.x = x;
		this.y = y;
	}

}
