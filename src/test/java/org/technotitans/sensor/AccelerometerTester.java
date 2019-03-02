package org.technotitans.sensor;

import frc.robot.sensors.gy521.Accel_GY521;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;

public class AccelerometerTester {

    private static final double EPSILON = 1e-6;
    private List<Accel_GY521> sensors;

    @Before
    public void setup() {
        Accel_GY521 elbow = mock(Accel_GY521.class);
        Accel_GY521 wrist = mock(Accel_GY521.class);
        this.sensors.add(elbow);
        this.sensors.add(wrist);
    }

//    @Test
//    public void sensorsShouldBeConnected() {
//        sensors.forEach(s -> {
//            assertTrue(s.isSensorConnected());
//        });
//    }

//    @Test
//    public void sensorsShouldOutputReadingsOnAccelerometer() {
//        // TODO Determine if this is a good way to test or not
//        // fails if the readings are exactly 0.0
//        sensors.forEach(s -> {
//            assertNotEquals(0.0, s.getX());
//            assertNotEquals(0.0, s.getY());
//            assertNotEquals(0.0, s.getZ());
//        });
//    }
//
//    @Test
//    public void sensorsGiveValidAngleAfterCalculations() {
//        sensors.forEach(s -> {
//            double angle = s.getAngle();
//            assertTrue(0.0 < angle && angle < 90.0);
//        });
//    }
}
