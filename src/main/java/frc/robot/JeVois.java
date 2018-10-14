package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

public class JeVois extends Thread {

    private SerialPort camera;
    private String lastString;

    public JeVois() {
        this(SerialPort.Port.kUSB);
    }

    public JeVois(SerialPort.Port port) { // port should be kUSB, kUSB1, or kUSB2
        this.camera = new SerialPort(921600, port);
    }

    @Override
    public void run() {
        try {
            lastString = camera.readString();
            System.out.println(lastString);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public String getLastString() {
        return lastString;
    }

}