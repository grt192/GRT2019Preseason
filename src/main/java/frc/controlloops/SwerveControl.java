package frc.controlloops;

import frc.fieldmapping.EncoderPositionTracker;
import frc.fieldmapping.PositionTracker;
import frc.swerve.FullSwerve;
import frc.swerve.SwerveData;

public class SwerveControl extends Thread {

	private static final long TIME_STEP = 10;
	private static final double dT = TIME_STEP / 1000.0;

	private FullSwerve swerve;
	private PositionPID thetaPID;

	private PositionTracker positionTracker;

	private volatile boolean reset;
	private volatile boolean enabled;

	private volatile double userVX, userVY, userW;

	public SwerveControl(FullSwerve swerve) {
		this.swerve = swerve;
		thetaPID = new PositionPID(8.5, 0.0, 5.7);
		thetaPID.setCyclical(0, Math.PI * 2);
		thetaPID.setOutputBounds(-1.0, 1.0);
		positionTracker = new EncoderPositionTracker(dT);
		reset = true;
	}

	@Override
	public void run() {
		boolean positionPIDenabled = false;
		positionTracker.reset();
		while (true) {
			long start = System.currentTimeMillis();
			if (reset)
				doEnable();
			SwerveData data = swerve.getSwerveData();
			positionTracker.update(data);
			if (enabled) {
				double w = userW;
				if (!positionPIDenabled && w == 0) {
					thetaPID.reset();
					thetaPID.setSetpoint(data.gyroAngle);
					positionPIDenabled = true;
				}
				if (positionPIDenabled) {
					if (w != 0) {
						positionPIDenabled = false;
					} else {
						w = thetaPID.calculate(data.gyroAngle, data.gyroW, dT);
					}
				}
				swerve.drive(userVX, userVY, w);
			}
			try {
				Thread.sleep(start + TIME_STEP - System.currentTimeMillis());
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	public void enable() {
		reset = true;
	}

	public void disable() {
		enabled = false;
	}

	private void doEnable() {
		reset = false;
		enabled = true;
		setVelocity(0, 0, 0);
	}

	public boolean isEnabled() {
		return enabled;
	}

	public void setVelocity(double vx, double vy) {
		userVX = vx;
		userVY = vy;
	}

	public void setVelocity(double vx, double vy, double w) {
		userVX = vx;
		userVY = vy;
		userW = w;
	}

	public void setAngle(double angle) {
		userW = 0;
		thetaPID.setSetpoint(angle);
	}

}
