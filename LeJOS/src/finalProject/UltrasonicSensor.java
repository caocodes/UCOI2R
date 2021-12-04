package finalProject;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

public class UltrasonicSensor {
    EV3UltrasonicSensor sensor;
    SampleProvider distMode;
    SampleProvider average;
    int distance;
    float[] sample;
    private final int DISTANCE_THRESHOLD = 0;

    public UltrasonicSensor(Port s1) {
        sensor = new EV3UltrasonicSensor(s1);
        distMode = sensor.getMode("Distance");
        distance = 255;
    }

    /**
     * @return average distance reading for 10 samples
     */
    public int distance() {
        float [] sample = new float[distMode.sampleSize()];
        distMode.fetchSample(sample, 0);
		average = new MeanFilter(distMode, 10);
		sample = new float[average.sampleSize()];
		average.fetchSample(sample, 0);
		distance = (int) (sample[0] * 1000);  // to easily filter out some noise 
        return distance;
    }

//    public void checkForWall() throws WallEncounteredException
//    {
//        if(distance() <= DISTANCE_THRESHOLD)
//        {
//            throw new WallEncounteredException("Wall Encountered!");
//        }
//    }
//
////    ir.close();
//    public class WallEncounteredException extends Exception
//    {
//        public WallEncounteredException(String errorMessage)
//        {
//            super(errorMessage);
//        }
//    }
}