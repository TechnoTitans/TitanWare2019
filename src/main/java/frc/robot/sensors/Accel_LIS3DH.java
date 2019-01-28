package frc.robot.sensors;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.robot.sensors.util.AccelerationInfo;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

import static frc.robot.sensors.AccelerometerConstants.*;
import static java.util.Objects.requireNonNull;

@SuppressWarnings("Duplicates")
public class Accel_LIS3DH implements Accelerometer, PIDSource {

    private final I2C i2c_conn;

    // TODO Test filtering base raw values

    // MARK - state variables
    private Range currentRange = Range.k2G; // set this as default range
    private double x_accel,
                   y_accel,
                   z_accel;

    private PIDSourceType pidSourceType = PIDSourceType.kDisplacement; // todo figure out what this is

    /**
     * Represents the adafruit LIS3DH accelerometer
     * @param device_addr - hex device_addr of the accelerometer sensor
     */
    public Accel_LIS3DH(int device_addr) {
        this.i2c_conn = new I2C(I2C.Port.kOnboard, device_addr);
        this.x_accel = 0;
        this.y_accel = 0;
        this.z_accel = 0;
        this.init();
    }

    private void init() {
        i2c_conn.write(CTRL_REG5, 0x80); // this reboots the device
        // 5 ms delay to wait for reboot
        try {
            Thread.sleep((long) 5);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // 0x47 is actually 0b0100_0111:
        // 0b0100 means HR / Normal / Low-power mode (50 Hz)
        // 0b0111 means enable axis x, y, and z respectively
        // consult pg 35
        i2c_conn.write(CTRL_REG1, 0x47);
    }

    /**
     * Tests whether or not the sensor is connected. Should be used in unit tests to ensure this sensor is functional
     * @return the state of connectivity of the sensor
     */
    public boolean isConnected() {
        return i2c_conn.verifySensor(REG_WHO_AM_I, 1, WHOAMI_DEFAULT_VALUE);
    }

    // for now, don't change range as it defaults to 2g (fs section of CTRL_REG4)
    // http://bitwisecmd.com/ helps as a visual bitwise calculator
    @Override
    public void setRange(Range range) {
        requireNonNull(range, "Hey you gave me a null range you buffoon. What did you expect?");

        // only 2 bits represent the range in the CTRL_REG4 (pg 37), the FS1 and FS0 bits,
        // but we have to read/write the whole register
        // therefore, we must first get the current value, modify it, then write that back to the device

        // todo test to see if this works
        ByteBuffer ctrl4Buffer = ByteBuffer.allocate(1);

        i2c_conn.read(CTRL_REG4, 1, ctrl4Buffer);
        ctrl4Buffer.order(ByteOrder.LITTLE_ENDIAN);

        byte ctrl4 = ctrl4Buffer.get();
        ctrl4 &= ~0x30; // this basically resets only the FS1 and FS0 bits
        ctrl4 |= getSettingFromRange(range) << 4; // this sets only the FS1 and FS0

        i2c_conn.write(CTRL_REG4, ctrl4);

        this.currentRange = range;
    }

    // Important info:
    // When you `read` with the provided I2C class, you are really just initiating a transaction
    // and sending the requested addr as the first byte of the transaction data
    // when you or 0x80 with a register value, you are essentially setting the MSB to 1
    // According to pg. 25, setting the MSB to 1 on a requested addr puts the device into auto-increment mode.
    // (ref pg.25)

    // adapted from https://github.com/adafruit/Adafruit_CircuitPython_LIS3DH/blob/master/adafruit_lis3dh.py
    private void updateAllValues() {
        double currentResolution = getCurrentResolution();
        ByteBuffer rawBuffer = ByteBuffer.allocate(6); // 3 axes * 2 bytes each (Low, High)
        // starts at the first register, OUT_X_L, then auto-increments until the last register, OUT_Z_H.
        i2c_conn.read(OUT_X_L | ENABLE_AUTOINCR, 6, rawBuffer);

        rawBuffer.order(ByteOrder.LITTLE_ENDIAN); // Low, High ordering means that LSB is first, so this is necessary
        System.out.println("The buffer is " + Arrays.toString(rawBuffer.array()));

        //  X_L  X_H  Y_L  Y_H  Z_L  Z_H
        //  0    1    2    3    4    5
        double rawX = rawBuffer.getShort(0); // creates a short from [0,1] (Short.BYTES == 2)
        double rawY = rawBuffer.getShort(2); // creates a short from [2,3]
        double rawZ = rawBuffer.getShort(4);

        this.x_accel = (rawX / currentResolution) /* * STANDARD_GRAVITY*/;
        this.y_accel = (rawY / currentResolution) /* * STANDARD_GRAVITY*/;
        this.z_accel = (rawZ / currentResolution) /* * STANDARD_GRAVITY*/;
    }

    // todo refactor duplicate code
    private int getCurrentResolution() {
        switch (currentRange) {
            case k16G:
                return RANGE_16_G_RES;
            case k8G:
                return RANGE_8_G_RES;
            case k4G:
                return RANGE_4_G_RES;
            case k2G:
                return RANGE_2_G_RES;

            default:
                throw new IllegalArgumentException("Invalid range given");
        }
    }

    private int getSettingFromRange(Range range) {
        switch (range) {
            case k2G:
                return SETTING_2G;
            case k4G:
                return SETTING_4G;
            case k8G:
                return SETTING_8G;
            case k16G:
                return SETTING_16G;
            default:
                throw new IllegalArgumentException("Invalid range given");
        }
    }

    @Override
    public double getX() {
        this.updateAllValues();
        return this.x_accel;
    }

    @Override
    public double getY() {
        this.updateAllValues();
        return this.y_accel;
    }

    @Override
    public double getZ() {
        this.updateAllValues();
        return this.z_accel;
    }

    public AccelerationInfo getAllAxes() {
        return new AccelerationInfo(this.getX(), this.getY(), this.getZ());
    }


    /**
     * Calculates the angle on the X-Y plane.
     * @return The angle (in degrees) at which the sensor is at currently on the XY plane
     */
    public double getAngleXY() {
        return Math.toDegrees(Math.atan2(this.getY(), this.getX()));
    }


    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        // kDisplacement - angle
        // kRate - rate of change of the angle (tbi)
        this.pidSourceType = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return this.pidSourceType;
    }

    @Override
    public double pidGet() {
        switch (pidSourceType) {
            case kDisplacement:
                return this.getAngleXY();
            case kRate:
                throw new IllegalStateException("PIDSourceType kRate is unsupported at this time."); // TODO calculated the rate of change of the angle instead
            default:
                throw new IllegalStateException("Invalid PIDSourceType at this point in execution");
        }
    }
}
