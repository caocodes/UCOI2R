package a4;

import java.util.ArrayList;
import java.util.List;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;
import lejos.utility.Delay;

public class Main {

	public static void main(String[] args) {
		A4Pilot a4Pilot = new A4Pilot();
		boolean captured = false;

		ColorSensor colorSensor = new ColorSensor(SensorPort.S4);
		UltrasonicSensor ussr = new UltrasonicSensor(SensorPort.S1);
		TheClaw theClaw = new TheClaw(MotorPort.C);
		Mast mast = new Mast(MotorPort.B);
		int distance = 255;

		int d = 0;

		mast.search();

//		theClaw.startTest();

		while (!Button.ESCAPE.isDown() && !captured) {
			a4Pilot.travel(0.5);
			Delay.msDelay(500);
			mast.startIncSearch();
			
			search: {
				for (int i = 0; i < 8; i++) {
					mast.incrementalSearch();
					d = ussr.distance();
					System.out.println(d);
					if (d == 0) {
						break search;
					}
				}
			}
			
			if (d == 0) {
				for (int i = 0; i < 3; i++) {
					Sound.playTone(500, 800);
					Delay.msDelay(200);
				}
				Sound.playTone(1000, 800);
			}

			if ((colorSensor.getColor() == Color.BLUE) || (colorSensor.getColor() == 7)
					|| (colorSensor.getColor() == 1)) {
				a4Pilot.travel(4.2);
				theClaw.grab();
				if (Math.abs(theClaw.checkRotation() - theClaw.closedRotation) < 5) {
					captured = true;
				}

			}

//			d++;
		}
		mast.mastMotor.rotateTo(0);

	}

//	public static void main(String[] args) {
//		A4Pilot a4Pilot = new A4Pilot();
//		boolean captured = false;
//
//		ColorSensor colorSensor = new ColorSensor(SensorPort.S4);
//		UltrasonicSensor ussr = new UltrasonicSensor(SensorPort.S1);
//		TheClaw theClaw = new TheClaw(MotorPort.B);
//		int distance = 255;
//
//
//
//		a4Pilot.forward();
//		while (!Button.ESCAPE.isDown() && !captured) {
//			distance = ussr.distance();
//
//			LCD.drawString("DIST:     ", 0, 2);
//			LCD.drawInt(distance, 3, 5, 2);
//
//			a4Pilot.forward();
//			if (colorSensor.getColor() == Color.BLUE) {
//				a4Pilot.travel(2.54);
//				theClaw.startTest();
//				
//			}
//
//			Delay.msDelay(100);
//		}
//		ussr.sensor.close();
//		colorSensor.sensor.close();
//
//	}

}