package frc.controlloops;

import java.util.BitSet;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonDataLoop extends Thread {

    public static final int POSITION_INDEX = 0;
    public static final int VELOCITY_INDEX = 1;

    private final long TIME_STEP;
    private TalonSRX talon;
    private BitSet flags;

    private volatile int position;
    private volatile int velocity;

    public TalonDataLoop(TalonSRX talon, long timeStep, BitSet flags) {
        TIME_STEP = timeStep;
        this.talon = talon;
        this.flags = flags;
    }

    public void run() {
        while (true) {
            long start = System.currentTimeMillis();
            if (flags.get(POSITION_INDEX))
                position = talon.getSelectedSensorPosition(0);
            if (flags.get(VELOCITY_INDEX))
                velocity = talon.getSelectedSensorVelocity(0);
            long end = System.currentTimeMillis();
            long sleepTime = start - end + TIME_STEP;
            if (sleepTime > 0) {
                try {
                    Thread.sleep(sleepTime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public int getPosition() {
        return position;
    }

    public int getVelocity() {
        return velocity;
    }
}