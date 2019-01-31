package frc.robot.sensors;

public class Accelerometer_Constants {
    public static final int WHO_AM_I = 0x75;
    public static final byte[] WHO_AM_I_DEFAULT = new byte[] {
      0b01101000
    };
    public static final byte RESET_DEVICE = 0b00000111;
    public static final double STANDARD_GRAVITY = 9.806;

    public static final int RESET_ADDRESS = 0x68;
    public static final int RESET_VAL = 0x07;
    public static final int ACCEL_RANGE = 0x1C;


    public static final int ACCEL_SETTING_2G = 0b00,
            ACCEL_SETTING_4G = 0b01,
            ACCEL_SETTING_8G = 0b10,
            ACCEL_SETTING_16G = 0b11;

    public static final int ACCEL_RESOLUTION_2G = 16384,
            ACCEL_RESOLUTION_4G = 8192,
            ACCEL_RESOLUTION_8G = 4096,
            ACCEL_RESOLUTION_16G = 2048;

    public static final int GYRO_SETTING_250 = 0b00,
            GYRO_SETTING_500 = 0b01,
            GYRO_SETTING_1000 = 0b10,
            GYRO_SETTING_2000 = 0b11;

    public static final int OUT_X_H = 0x3B,
            OUT_X_L = 0x3C,
            OUT_Y_H = 0x3D,
            OUT_Y_L = 0x3E,
            OUT_Z_H = 0x3F,
            OUT_Z_L = 0x3F;
}
