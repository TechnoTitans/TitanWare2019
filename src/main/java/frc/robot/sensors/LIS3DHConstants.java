package frc.robot.sensors;

public class LIS3DHConstants {

    // MARK - Universal Constants

    // from https://github.com/adafruit/Adafruit_CircuitPython_LIS3DH/blob/master/adafruit_lis3dh.py
    public static final double STANDARD_GRAVITY = 9.806;

    // MARK - register constant values
    public static final byte[] WHOAMI_DEFAULT_VALUE = new byte[] {
            0b00110011
    };

    public static final int ENABLE_AUTOINCR = 0x80;


    // MARK - Registers
    public static final int REG_WHO_AM_I = 0x0F;

    public static final int CTRL_REG1 = 0x20;
    public static final int CTRL_REG2 = 0x21;
    public static final int CTRL_REG3 = 0x22;
    public static final int CTRL_REG4 = 0x23;

    public static final int OUT_X_L = 0x28,
            OUT_X_H = 0x29,
            OUT_Y_L = 0x2A,
            OUT_Y_H = 0x2B,
            OUT_Z_L = 0x2C,
            OUT_Z_H = 0x2D;

    // these are what the raw values need to be divided by in order to get the real value
    // res stands for resolution
    public static final int RANGE_16_G_RES = 1365,
                            RANGE_8_G_RES = 4096,
                            RANGE_4_G_RES = 8190,
                            RANGE_2_G_RES = 16380;

    public static final int SETTING_2G = 0b00,
                            SETTING_4G = 0b01,
                            SETTING_8G = 0b10,
                            SETTING_16G = 0b11; // pg 37 of manual
    
}
