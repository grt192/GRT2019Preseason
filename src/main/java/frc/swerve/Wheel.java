package frc.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.config.Config;
import frc.util.GRTUtil;

class Wheel {

	private final double TICKS_PER_ROTATION;
	private int OFFSET;
	private final double DRIVE_TICKS_TO_METERS;

	private static final double TWO_PI = Math.PI * 2;

	private static final double kP = 9000.0;
	private static final double kI = 0.0;
	private static final double kD = 0.0;
	private static final double maxIAccum = 0.0;

	private TalonSRX rotateMotor;
	private TalonSRX driveMotor;

	private String name;

	private boolean reversed;

	public Wheel(String name) {
		this.name = name;

		rotateMotor = new TalonSRX(Config.getInt(name + "_rotate"));
		driveMotor = new TalonSRX(Config.getInt(name + "_drive"));
		TICKS_PER_ROTATION = Config.getDouble("ticks_per_rotation");
		OFFSET = Config.getInt(name + "_offset");
		DRIVE_TICKS_TO_METERS = Config.getDouble("drive_encoder_scale");

		configRotateMotor();
		configDriveMotor();
	}

	public void enable() {
		rotateMotor.set(ControlMode.Disabled, 0);
		set(0, 0);
	}

	public void zero() {
		System.out.println("Zeroing " + name + "module");
		OFFSET = rotateMotor.getSelectedSensorPosition(0);
	}

	public void disable() {
		rotateMotor.set(ControlMode.Disabled, 0);
		driveMotor.set(ControlMode.Disabled, 0);
	}

	public void set(double radians, double speed) {
		if (speed != 0.0) {
			double targetPosition = radians / TWO_PI;
			targetPosition = GRTUtil.positiveMod(targetPosition, 1.0);

			int encoderPosition = rotateMotor.getSelectedSensorPosition(0) - OFFSET;
			double currentPosition = encoderPosition / TICKS_PER_ROTATION;
			double rotations = Math.floor(currentPosition);
			currentPosition -= rotations;
			double delta = currentPosition - targetPosition;
			if (Math.abs(delta) > 0.5) {
				// targetPosition += Math.signum(delta);
				delta -= Math.signum(delta);
			}
			// delta = currentPosition - targetPosition;
			boolean newReverse = false;
			if (Math.abs(delta) > 0.25) {
				targetPosition += Math.signum(delta) * 0.5;
				newReverse = true;
			}
			targetPosition += rotations;
			reversed = newReverse;
			double encoderPos = targetPosition * TICKS_PER_ROTATION + OFFSET;
			rotateMotor.set(ControlMode.Position, encoderPos);
		}
		speed *= (reversed ? -1 : 1);
		driveMotor.set(ControlMode.Velocity, speed);
	}

	public double getDriveSpeed() {
		return driveMotor.getSelectedSensorVelocity(0) * DRIVE_TICKS_TO_METERS * 10 * (reversed ? -1 : 1);
	}

	public double getCurrentPosition() {
		return GRTUtil.positiveMod((((rotateMotor.getSelectedSensorPosition(0) - OFFSET) * TWO_PI / TICKS_PER_ROTATION)
				+ (reversed ? Math.PI : 0)), TWO_PI);
	}

	public String getName() {
		return name;
	}

	private void configRotateMotor() {
		Config.defaultConfigTalon(rotateMotor);

		boolean inverted = Config.getBoolean("swerve_inverted") ^ Config.getBoolean(name + "_inverted");
		rotateMotor.setInverted(inverted);
		rotateMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
		rotateMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 0);

		rotateMotor.config_kP(0, kP / TICKS_PER_ROTATION, 0);
		rotateMotor.config_kI(0, kI / TICKS_PER_ROTATION, 0);
		rotateMotor.config_kD(0, kD / TICKS_PER_ROTATION, 0);
		rotateMotor.config_kF(0, 0, 0);
		rotateMotor.configMaxIntegralAccumulator(0, maxIAccum, 0);
		rotateMotor.configAllowableClosedloopError(0, 0, 0);
	}

	private void configDriveMotor() {
		Config.defaultConfigTalon(driveMotor);

		driveMotor.configContinuousCurrentLimit(40, 0);
		driveMotor.configPeakCurrentDuration(0, 0);
		driveMotor.enableCurrentLimit(true);
		driveMotor.configOpenloopRamp(0, 0);

		driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 0);
		driveMotor.getSensorCollection().setQuadraturePosition(0, 0);

		driveMotor.config_kP(0, Config.getDouble("velocity_p") / DRIVE_TICKS_TO_METERS, 0);
		driveMotor.config_kI(0, 0, 0);
		driveMotor.config_kD(0, 0, 0);
		driveMotor.config_kF(0, Config.getDouble("velocity_f") / DRIVE_TICKS_TO_METERS, 0);
	}

}
