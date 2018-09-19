package frc.controlloops;

import frc.util.GRTUtil;

class PositionPID {

	private double kP, kI, kD;
	private double maxAccum;
	private double minOutput, maxOutput;
	private double minInput, inputRange;
	private boolean cyclical;

	private double prevSensor;
	private double prevError;
	private double totalError;
	private boolean started;
	private volatile double setpoint;

	public PositionPID(double p, double i, double d) {
		kP = p;
		kI = i;
		kD = d;
		minOutput = Double.NEGATIVE_INFINITY;
		maxOutput = Double.POSITIVE_INFINITY;
		maxAccum = Double.POSITIVE_INFINITY;
		started = false;
	}

	public static PositionPID industryForm(double p, double ti, double td) {
		return new PositionPID(p, p / ti, p * td);
	}

	public void setOutputBounds(double min, double max) {
		minOutput = min;
		maxOutput = max;
	}

	public void setCyclical(double min, double max) {
		minInput = min;
		maxOutput = max;
		inputRange = max - min;
		cyclical = true;
	}

	public void setMaxAccum(double max) {
		maxAccum = max;
	}

	public void reset() {
		totalError = 0;
		started = false;
	}

	public void setSetpoint(double set) {
		if (cyclical) {
			setpoint = modIntoRange(set);
		} else {
			setpoint = set;
		}
	}

	public double calculate(double pv, double dt) {
		double rate = 0;
		if (started) {
			rate = (pv - prevSensor) / dt;
		}
		return calculate(pv, rate, dt);
	}

	public double calculate(double pv, double rate, double dt) {
		double error = getError(pv);
		if (started) {
			totalError += dt * (prevError + error) / 2.0;
			totalError = GRTUtil.clamp(-maxAccum, totalError, maxAccum);
		}
		rate *= -1; // Rate should be dE/dt = -1 * dPV/dt
		double output = kP * error + kI * totalError + kD * rate;
		prevSensor = pv;
		prevError = error;
		started = true;
		return GRTUtil.clamp(minOutput, output, maxOutput);
	}

	public double getError(double pv) {
		if (!cyclical)
			return setpoint - pv;
		pv = modIntoRange(pv);
		double error = setpoint - pv;
		if (Math.abs(error) > inputRange / 2) {
			error -= Math.signum(error) * inputRange;
		}
		return error;
	}

	private double modIntoRange(double x) {
		return GRTUtil.positiveMod(x - minInput, inputRange) + minInput;
	}

}
