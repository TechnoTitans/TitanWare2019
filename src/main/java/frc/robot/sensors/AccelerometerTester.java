package frc.robot.sensors;

import edu.wpi.first.wpilibj.I2C;

import static frc.robot.sensors.lis3dh.LIS3DHConstants.WHOAMI_DEFAULT_VALUE;

public class AccelerometerTester {



    public static final int ACCEL_REGISTER = 0x0F;

    public static boolean isSensorAccessable() {
        I2C i2C = new I2C(I2C.Port.kOnboard, 0x18);
        return i2C.verifySensor(0x0F, 1, WHOAMI_DEFAULT_VALUE);
    }

}
