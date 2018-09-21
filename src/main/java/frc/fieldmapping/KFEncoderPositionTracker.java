package frc.fieldmapping;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.video.KalmanFilter;

import frc.swerve.SwerveData;

public class KFEncoderPositionTracker {

    private static final int TYPE = CvType.CV_64F;
    private static final int STATES = 4;
    private static final int MEASURES = 2;
    private static final int CONTROLS = 2;

    private double dt;
    private KalmanFilter kf;
    private Mat state;

    public KFEncoderPositionTracker(double dt) {
        this.dt = dt;
        kf = createKF();
    }

    public void update(double vx, double vy, SwerveData data) {
        Mat z = new Mat(MEASURES, 0, TYPE);
        z.put(0, 0, data.encoderVX, data.encoderVY);
        state = kf.correct(z);
        Mat u = new Mat(CONTROLS, 1, TYPE);
        double[] predV = new double[2];
        state.get(2, 0, predV);
        u.put(0, 0, vx - predV[0], vy - predV[1]);
        kf.predict(u);
    }

    public double getX() {
        return state.get(0, 0)[0];
    }

    public double getY() {
        return state.get(1, 0)[0];
    }

    private KalmanFilter createKF() {
        KalmanFilter kf = new KalmanFilter(STATES, MEASURES, CONTROLS, TYPE);
        kf.set_transitionMatrix(createF());
        kf.set_controlMatrix(createB());
        kf.set_measurementMatrix(createH());
        kf.set_processNoiseCov(createQ());
        kf.set_measurementNoiseCov(createR());

        kf.set_statePre(createInitialX());
        kf.set_statePost(createInitialX());

        kf.set_errorCovPre(createInitialP());
        kf.set_errorCovPost(createInitialP());
        state = kf.get_errorCovPost();
        return kf;
    }

    private Mat createF() {
        Mat F = new Mat(STATES, STATES, TYPE);
        F.put(0, 0, 1.0, 0.0, dt, 0.0);
        F.put(1, 0, 0.0, 1.0, 0.0, dt);
        F.put(2, 0, 0.0, 0.0, 1.0, 0.0);
        F.put(3, 0, 0.0, 0.0, 0.0, 1.0);
        return F;
    }

    private Mat createB() {
        Mat B = new Mat(STATES, CONTROLS, TYPE);
        B.put(0, 0, 0.0, 0.0);
        B.put(1, 0, 0.0, 0.0);
        B.put(2, 0, 1.0, 0.0);
        B.put(3, 0, 0.0, 1.0);
        return B;
    }

    private Mat createH() {
        Mat H = new Mat(MEASURES, STATES, TYPE);
        H.put(0, 0, 0.0, 0.0, 1.0, 0.0);
        H.put(1, 0, 0.0, 0.0, 0.0, 1.0);
        return H;
    }

    private Mat createQ() {
        Mat Q = new Mat(STATES, STATES, TYPE);
        Q.put(0, 0, 0.01, 0.0, 0.01, 0.0);
        Q.put(1, 0, 0.0, 0.01, 0.0, 0.01);
        Q.put(2, 0, 0.01, 0.0, 0.05, 0.05);
        Q.put(3, 0, 0.0, 0.01, 0.05, 0.05);
        return Q;
    }

    private Mat createR() {
        Mat R = new Mat(MEASURES, MEASURES, TYPE);
        R.put(0, 0, 0.1, 0.1);
        R.put(1, 0, 0.1, 0.1);
        return R;
    }

    private Mat createInitialX() {
        return Mat.zeros(STATES, 1, TYPE);
    }

    private Mat createInitialP() {
        Mat P = new Mat(STATES, STATES, TYPE);
        P.put(0, 0, 0.01, 0.0, 0.0, 0.0);
        P.put(1, 0, 0.0, 0.01, 0.0, 0.0);
        P.put(2, 0, 0.0, 0.0, 0.0, 0.0);
        P.put(3, 0, 0.0, 0.0, 0.0, 0.0);
        return P;
    }
}