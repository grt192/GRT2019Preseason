package frc.mechs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import frc.config.Config;

public class Elevator {

	private TalonSRX elevator;
	private TalonSRX follower;
	private Solenoid winchLock;

	public Elevator() {
		elevator = new TalonSRX(Config.getInt("winch_motor"));
		elevator.setNeutralMode(NeutralMode.Brake);
		follower = new TalonSRX(Config.getInt("winch_motor_follower"));
		follower.setNeutralMode(NeutralMode.Brake);
		winchLock = new Solenoid(Config.getInt("winchsol"));
		follower.set(ControlMode.Follower, elevator.getDeviceID());
	}

	public void setPosition(double height) {
		elevator.set(ControlMode.Position, height);
	}

	public int getHeight() {
		return elevator.getSelectedSensorPosition(0);
	}

	public void setSpeed(double speed) {
		elevator.set(ControlMode.PercentOutput, speed);
	}

	public void breakElevator() {
		winchLock.set(!winchLock.get());
	}

	public double getSpeed() {
		return elevator.getSelectedSensorVelocity(0);
	}

}
