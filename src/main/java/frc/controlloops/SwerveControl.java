package frc.controlloops;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.fieldmapping.EncoderPositionTracker;
import frc.robot.JoystickProfile;
import frc.swerve.FullSwerve;
import frc.swerve.SwerveData;

public class SwerveControl extends Thread {

	private static final long TIME_STEP = 10;
	private static final double dT = TIME_STEP / 1000.0;
	private static final double MAX_SPEED = 2.95;
	private static final double MAX_ANGULAR_SPEED = 6.95;

	private FullSwerve swerve;
	private PositionPID thetaPID;

	private EncoderPositionTracker positionTracker;

	private volatile boolean reset;
	private volatile boolean enabled;

	private volatile double userVX, userVY;
	private volatile double userW;

	public SwerveControl(FullSwerve swerve) {
		setPriority(MAX_PRIORITY);
		this.swerve = swerve;
		thetaPID = new PositionPID(0.8, 0, 0);// new PositionPID(0.86, 0.0, 0.57);
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
			long start = System.nanoTime();
			nextLoop += TIME_STEP;
			if (reset)
				doEnable();
			SwerveData data = swerve.getSwerveData();
			positionTracker.update(data);
			SmartDashboard.putNumber("x", positionTracker.getX());
			SmartDashboard.putNumber("y", positionTracker.getY());
			SmartDashboard.putNumber("gyro", data.gyroAngle);
			if (enabled) {
				double vx = userVX;
				double vy = userVY;
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
				System.out.println("vx: " + vx + "; vy: " + vy + "; w: " + w);
				swerve.drive(vx, vy, w);
			}
			long sleepTime = nextLoop - System.currentTimeMillis();
			System.out.println("Total time: " + (System.nanoTime() - start) + "ns");
			if (sleepTime > 0) {
				try {
					Thread.sleep(sleepTime);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			} else {
				System.out.println("Swerve loop too slow!!!");
			}
			System.out.println("With sleep: " + (System.nanoTime() - start) + "ns");
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
		userVX = vx * MAX_SPEED;
		userVY = vy * MAX_SPEED;
	}

	public void setVelocity(double vx, double vy, double w) {
		setVelocity(vx, vy);
		userW = w * MAX_ANGULAR_SPEED;
	}

	public void setAngle(double angle) {
		userW = 0;
		thetaPID.setSetpoint(angle);
	}

}
