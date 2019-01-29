package frc.robot.sensors;

public class Accelerometer_Constants {
    public static final int WHO_AM_I = 0x75;
    public static final byte[] WHO_AM_I_DEFAULT = new byte[] {
      0b01101000
    };

    public static final int RESET_ADDRESS = 0x68;
    public static final int RESET_VAL = 0x07;
    public static final int ACCEL_RANGE = 0x1C;

    public static final int ACCEL_SETTING_2G = 0b00,
            ACCEL_SETTING_4G = 0b01,
            ACCEL_SETTING_8G = 0b10,
            ACCEL_SETTING_16G = 0b11;

    public static final int GYRO_SETTING_250 = 0b00,
            GYRO_SETTING_500 = 0b01,
            GYRO_SETTING_1000 = 0b10,
            GYRO_SETTING_2000 = 0b11;
}
