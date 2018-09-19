package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import frc.controlloops.SwerveControl;
import frc.mechs.MechCollection;

public class Teleop {

	private SwerveControl swerve;
	private MechCollection mechs;
	private XboxController mechXbox;
	private XboxController swerveXbox;

	public Teleop(SwerveControl swerve, MechCollection mechs) {
		swerveXbox = new XboxController(0);
		mechXbox = new XboxController(1);
		this.swerve = swerve;
		this.mechs = mechs;
	}

	public void init() {

	}

	public void periodic() {
		manualControls();
	}

	private void manualControls() {
		if (mechXbox.getXButtonPressed()) {
			mechs.intake.moveCenterPickup();
		}
		if (mechXbox.getBumperPressed(Hand.kRight)) {
			mechs.intake.movePickup();
		}
		mechs.intake.moveWheels(mechXbox.getY(Hand.kLeft));

		if (mechXbox.getBumperPressed(Hand.kLeft)) {
			mechs.elevator.breakElevator();
		}

		double elevatorSpeed = JoystickProfile.applyDeadband(-mechXbox.getY(Hand.kRight));
		mechs.elevator.setSpeed(elevatorSpeed);
		if (elevatorSpeed != 0.0) {
			mechs.intake.movePickupOut();
		}

		driveSwerve();
	}

	private void driveSwerve() {
		double x = JoystickProfile.clipAndSquare(-swerveXbox.getY(Hand.kLeft));
		double y = JoystickProfile.clipAndSquare(swerveXbox.getX(Hand.kLeft));
		double lTrigger = swerveXbox.getTriggerAxis(Hand.kLeft);
		double rTrigger = swerveXbox.getTriggerAxis(Hand.kRight);
		double rotate = 0;
		if (lTrigger + rTrigger > 0.05) {
			rotate = rTrigger * rTrigger - lTrigger * lTrigger;
		} else {
			double rx = -swerveXbox.getY(Hand.kRight);
			double ry = swerveXbox.getX(Hand.kRight);
			if (rx * rx + ry * ry > 0.7) {
				double theta = Math.atan2(ry, rx);
				swerve.setAngle(theta);
			}
		}
		swerve.setVelocity(x, y, rotate);
	}

}
