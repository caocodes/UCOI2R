package TheJOS;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MoveController;
import lejos.robotics.navigation.MovePilot;

public class MyPilot extends MovePilot{
	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);

	public MyPilot(double wheelOffset, double linearSpeed, double angularSpeed) {
		super(initChassis(wheelOffset));
		setLinearSpeed(linearSpeed);
		setAngularSpeed(angularSpeed);
		colorSensor.getColorIDMode();
	}

	static Chassis initChassis(double wheelOffset) {
		double wheelDiam = MoveController.WHEEL_SIZE_NXT1;
		EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.C);
		EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.B);
		Wheel leftWheel = WheeledChassis.modelWheel(leftMotor, wheelDiam).offset(-1*wheelOffset);
		Wheel rightWheel = WheeledChassis.modelWheel(rightMotor, wheelDiam).offset(wheelOffset);
		return new WheeledChassis(new Wheel[] { leftWheel, rightWheel }, WheeledChassis.TYPE_DIFFERENTIAL);
	}
		
	public void turn(double speed, double angle, double radius) {
		setMinRadius(radius);
		setAngularSpeed(speed);
		rotate(angle);
	}
	
	public boolean detectColor(int color) {
		return colorSensor.getColorID() == color;
	}
}