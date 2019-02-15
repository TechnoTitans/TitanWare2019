package frc.robot.sensors.gy521;

@SuppressWarnings("WeakerAccess")
public class GY521_Constants {
    public static final int WHO_AM_I = 0x75;
    // https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
    public static final byte[] WHO_AM_I_DEFAULT = new byte[] {
//      0b01101000
      0x68 // page 45
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

    // MARK - accelerometer address values
    public static final int ACCEL_XOUT_H = 0x3B,
                            ACCEL_YOUT_H = 0x3D,
                            ACCEL_ZOUT_H = 0x3F;

    // MARK - gyroscope address values
    // TODO implement x and y address
    public static final int GYRO_ZOUT_H = 0x47;

    // MARK - Gyro Scale Modifiers
    public static final double GYRO_SCALE_MODIFIER_250DEG = 131.0,
                               GYRO_SCALE_MODIFIER_500DEG = 65.5,
                               GYRO_SCALE_MODIFIER_1000DEG = 32.8,
                               GYRO_SCALE_MODIFIER_2000DEG = 16.4;

    // MARK - Configuration addresses
    public static final int GYRO_CONFIG  = 0x1B,
                            ACCEL_CONFIG = 0x1C;


    public static final int PWR_MGMT_1 = 0x6B;
}
