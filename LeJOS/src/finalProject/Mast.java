package finalProject;

import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;

public class Mast {

	private int minRotation = -55;
	private int maxRotation = 55;
//	private int deltaRotation;
	private int currentRotation;
	public EV3MediumRegulatedMotor mastMotor;
	private Port mastPort;
	
	// Initialize sonar mast
	public Mast(Port c) {
		mastPort = c;
		mastMotor = new EV3MediumRegulatedMotor(c);
		currentRotation = 0;
	}
	
	// rotation getter
	public int checkRotation() {
		currentRotation = mastMotor.getLimitAngle();
		return currentRotation;
	}
	
	// future search pattern work
	public void search() {
		mastMotor.rotateTo(minRotation);
		int searchInterval = 8;
		int searchArc = maxRotation-minRotation;
		searchArc /= searchInterval;
		for (int i = 1; i <= searchInterval; i++) {
			mastMotor.rotate(searchArc);
		}
		mastMotor.rotateTo(0);
	}
	
	//reset rotation to the left
	public void startIncSearch() {
		mastMotor.rotateTo(minRotation);
	}
	
	// start looking right
	public void incrementalSearch() {
		if (currentRotation>=maxRotation) {
			mastMotor.rotateTo(minRotation);
		}
		else {
			mastMotor.rotate(5);
		}
	}
	
	//TODO soups check pls
	public void lookFront() {
		mastMotor.rotateTo(0);
	}
	
	//TODO soups check pls
	public void lookLeft() {
		mastMotor.rotateTo(-180);
	}
	
	//TODO soups check pls
	public void lookRight() {
		mastMotor.rotateTo(180);
	}
}
