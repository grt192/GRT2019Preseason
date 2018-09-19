package frc.controlloops;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
		thetaPID = new PositionPID(0.86, 0.0, 0.57);
		thetaPID.setCyclical(0, Math.PI * 2);
		thetaPID.setOutputBounds(-1.0, 1.0);
		positionTracker = new EncoderPositionTracker(dT);
		enabled = false;
	}

	@Override
	public void run() {
		boolean positionPIDenabled = false;
		positionTracker.reset();
		long nextLoop = System.currentTimeMillis();
		while (true) {
			nextLoop += TIME_STEP;
			if (reset)
				doEnable();
			SwerveData data = swerve.getSwerveData();
			positionTracker.update(data);
			SmartDashboard.putNumber("x", positionTracker.getX());
			SmartDashboard.putNumber("y", positionTracker.getY());
			SmartDashboard.putNumber("gyro", data.gyroAngle);
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
				System.out.println("vx: " + data.encoderVX + "; vy: " + data.encoderVY);
				swerve.drive(userVX, userVY, w);
			}
			long sleepTime = nextLoop - System.currentTimeMillis();
			if (sleepTime > 0) {
				try {
					Thread.sleep(sleepTime);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
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