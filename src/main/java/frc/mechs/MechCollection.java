package frc.mechs;

public class MechCollection {

	public final Elevator elevator;
	public final Intake intake;

	public MechCollection() {
		elevator = new Elevator();
		intake = new Intake();
	}

}
