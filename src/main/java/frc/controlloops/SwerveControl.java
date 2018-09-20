package frc.controlloops;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.fieldmapping.EncoderPositionTracker;
import frc.fieldmapping.PositionTracker;
import frc.robot.JoystickProfile;
import frc.swerve.FullSwerve;
import frc.swerve.SwerveData;
import frc.util.GRTUtil;

public class SwerveControl extends Thread {

	private static final long TIME_STEP = 10;
	private static final double dT = TIME_STEP / 1000.0;
	private static final double MAX_SPEED = 2.95;

	private FullSwerve swerve;
	private PositionPID thetaPID;
	private VelocityPIF xPIF, yPIF;

	private PositionTracker positionTracker;

	private volatile boolean reset;
	private volatile boolean enabled;

	private volatile double userW;

	public SwerveControl(FullSwerve swerve) {
		setPriority(MAX_PRIORITY);
		this.swerve = swerve;
		thetaPID = new PositionPID(0.8, 0, 0);// new PositionPID(0.86, 0.0, 0.57);
		thetaPID.setCyclical(0, Math.PI * 2);
		thetaPID.setOutputBounds(-1.0, 1.0);
		xPIF = new VelocityPIF(0.34, 0.0, 0.5);
		xPIF.setMaxAccum(2.0);
		yPIF = new VelocityPIF(0.34, 0.0, 0.5);
		yPIF.setMaxAccum(2.0);
		positionTracker = new EncoderPositionTracker(dT);
		enabled = false;
	}

	@Override
	public void run() {
		boolean positionPIDenabled = false;
		positionTracker.reset();
		long nextLoop = System.currentTimeMillis();
		while (true) {
			long t1 = System.nanoTime();
			long t2 = 0;
			nextLoop += TIME_STEP;
			if (reset)
				doEnable();
			// print
			t2 = System.nanoTime();
			System.out.println("Reset took " + (t2 - t1) + "ns");
			SwerveData data = swerve.getSwerveData();
			// print
			t1 = System.nanoTime();
			System.out.println("Data took " + (t1 - t2) + "ns");
			positionTracker.update(data);
			// print
			t2 = System.nanoTime();
			System.out.println("Pos tracking took " + (t2 - t1) + "ns");
			SmartDashboard.putNumber("x", positionTracker.getX());
			SmartDashboard.putNumber("y", positionTracker.getY());
			SmartDashboard.putNumber("gyro", data.gyroAngle);
			// print
			t1 = System.nanoTime();
			System.out.println("SmartDashboard took " + (t1 - t2) + "ns");
			if (enabled) {
				double vx = xPIF.calculate(data.encoderVX, dT);
				double vy = yPIF.calculate(data.encoderVY, dT);
				vx = JoystickProfile.applyDeadband(vx);
				vy = JoystickProfile.applyDeadband(vy);
				// print
				t2 = System.nanoTime();
				System.out.println("Velocity took " + (t2 - t1) + "ns");
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
				// print
				t1 = System.nanoTime();
				System.out.println("Rotation took " + (t1 - t2) + "ns");
				System.out.println("vx: " + vx + "; vy: " + vy + "; w: " + w);
				swerve.drive(vx, vy, w);
				// print
				t2 = System.nanoTime();
				System.out.println("Driving took " + (t2 - t1) + "ns");
			}
			long sleepTime = nextLoop - System.currentTimeMillis();
			if (sleepTime > 0) {
				try {
					Thread.sleep(sleepTime);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			} else {
				System.out.println("Swerve loop too slow!!!");
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
		xPIF.reset();
		yPIF.reset();
		setVelocity(0, 0, 0);
	}

	public boolean isEnabled() {
		return enabled;
	}

	public void setVelocity(double vx, double vy) {
		xPIF.setSetpoint(vx * MAX_SPEED);
		yPIF.setSetpoint(vy * MAX_SPEED);
	}

	public void setVelocity(double vx, double vy, double w) {
		setVelocity(vx, vy);
		userW = w;
	}

	public void setAngle(double angle) {
		userW = 0;
		thetaPID.setSetpoint(angle);
	}

}
