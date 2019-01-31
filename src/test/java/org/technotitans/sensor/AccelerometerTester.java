//package org.technotitans.sensor;
//
//import edu.wpi.first.wpilibj.I2C;
//import org.junit.Test;
//
//
//import static org.junit.Assert.assertTrue;
//
//public class AccelerometerTester {
//
//    @Test
//    public void testElbowSensor() {
//        I2C elbowI2C_conn = new I2C(I2C.Port.kOnboard, RobotMap.ELBOW_ACCEL_ADDR);
//        assertTrue(elbowI2C_conn.verifySensor(REG_WHO_AM_I, 1, WHOAMI_DEFAULT_VALUE));
//    }
//
//    @Test
//    public void testWristSensor() {
//        I2C wristI2C_conn = new I2C(I2C.Port.kOnboard, RobotMap.WRIST_ACCEL_ADDR);
//        assertTrue(wristI2C_conn.verifySensor(REG_WHO_AM_I, 1, WHOAMI_DEFAULT_VALUE)); // TODO correctly implement the second sensor
//    }
//
//}
