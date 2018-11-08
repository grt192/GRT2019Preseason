package frc.robot;

import frc.controlloops.SwerveControl;
import frc.mechs.MechCollection;

public class Autonomous implements Runnable {

	private SwerveControl swerve;
	private MechCollection mechs;
	private Thread thread;

	public Autonomous(SwerveControl swerve, MechCollection mechs) {
		this.swerve = swerve;
		this.mechs = mechs;
	}

	public void init() {
		thread = new Thread(this);
		thread.start();
	}

	public void run() {
		try {
			runAutonomous();
		} catch (InterruptedException e) {
			System.out.println("Interrupting autonomous");
		}
	}

	public void disable() {
		if (thread != null)
			thread.interrupt();
	}

	private void runAutonomous() throws InterruptedException {
		// setup and choose auton here
		switchAuton();
	}

	private void switchAuton() throws InterruptedException {
		// Do auton
		moveToPosition(10, 5, 0.5, 0.1);
	}

	private void moveToPosition(double x, double y, double speed, double threshold) throws InterruptedException {
		double distance = Double.MAX_VALUE;
		while (distance > threshold) {
			double xPos = swerve.getTracker().getX();
			double yPos = swerve.getTracker().getY();
			double dX = x - xPos;
			double dY = y - yPos;
			distance = Math.sqrt(dX * dX + dY * dY);
			double vX = speed * dX / distance;
			double vY = speed * dX / distance;
			swerve.setVelocity(vX, vY);
			Thread.sleep(10);
		}

	}

	public void periodic() {

	}

}
