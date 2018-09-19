package frc.controlloops;

import frc.util.GRTUtil;

class VelocityPIF {

	private double kP, kI, kF;
	private double maxAccum;
	private double minOutput, maxOutput;

	private double prevError;
	private double totalError;
	private boolean started;
	private volatile double setpoint;

	public VelocityPIF(double p, double i, double f) {
		kP = p;
		kI = i;
		kF = f;
		minOutput = Double.NEGATIVE_INFINITY;
		maxOutput = Double.POSITIVE_INFINITY;
		maxAccum = Double.POSITIVE_INFINITY;
		started = false;
	}

	public void setOutputBounds(double min, double max) {
		minOutput = min;
		maxOutput = max;
	}

	public void setMaxAccum(double max) {
		maxAccum = max;
	}

	public void reset() {
		totalError = 0;
		started = false;
	}

	public void setSetpoint(double set) {
		setpoint = set;
	}

	public double calculate(double pv, double dt) {
		double error = setpoint - pv;
		if (started) {
			double tempOut = kP * error + kI * totalError + kF * setpoint;
			if (GRTUtil.inRange(minOutput, tempOut, maxOutput)) {
				totalError += dt * (prevError + error) / 2.0; // only accumulate if not already maxed
			}
			totalError = GRTUtil.clamp(-maxAccum, totalError, maxAccum);
		}
		double output = kP * error + kI * totalError + kF * setpoint;
		return GRTUtil.clamp(minOutput, output, maxOutput);
	}

}
