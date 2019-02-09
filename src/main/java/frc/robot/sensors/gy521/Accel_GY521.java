package frc.robot.sensors.gy521;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.interfaces.sensor.WatchedSensor;
import frc.robot.sensors.util.I2CUtils;

import java.nio.ByteBuffer;

import static frc.robot.sensors.gy521.GY521_Constants.*;
import static java.util.Objects.requireNonNull;

@SuppressWarnings({"Duplicates", "UnnecessaryLocalVariable", "SameParameterValue"})
public class Accel_GY521 extends WatchedSensor implements Accelerometer, Gyro {

    // MARK - complimentary filter initialization
    private static final double kGyroInfluence = 0.98;
    private double previousAngle = 0.0;
    private Timer timer;

    private I2C i2c_conn;
    private Range currRange;
    private int deviceAddr;

    // MARK - watchdog config
    private Watchdog watchdog;
    private static final double kSensorDisconnectTimeout = 3; // sec // todo better name?
    private boolean watchdogEnabled;


    // TODO get link to manual
    // TODO Create an init sendable

    //initializes all values
    public Accel_GY521(int address, boolean watchdogEnabled) {
        // device setup
        this.deviceAddr = address;
        this.watchdogEnabled = watchdogEnabled;
        i2c_conn = new I2C(I2C.Port.kOnboard, address);
        currRange = Range.k2G;
        this.resetDevice();
        this.setSleepMode(false);

        this.timer = new Timer();

        // watchdog setup
        this.watchdog = new Watchdog(kSensorDisconnectTimeout, this::reconnectDevice);
        this.initWatchdog();

    }


    // MARK - instance configuration methods

    @Override
    //sets the range for the accelerometer
    public void setRange(Range range) {
        //Ensures that the range is not null pointer
        requireNonNull(range, "The range that has been entered is invalid");

        ByteBuffer ctrl4Buffer = ByteBuffer.allocate(1);
        i2c_conn.read(ACCEL_RANGE, 1, ctrl4Buffer);

        byte ctrl4 = ctrl4Buffer.get();
        ctrl4 &= ~0x18; // this basically resets only the FS1 and FS0 bits
        ctrl4 |= getSettingFromRange(range) << 3; // this sets only the FS1 and FS0

        i2c_conn.write(ACCEL_RANGE, ctrl4);

        this.currRange = range;
    }

    //gets the resolution
    private int getCurrentResolution() {
        switch (currRange) {
            case k16G:
                return ACCEL_RESOLUTION_16G;
            case k8G:
                return ACCEL_RESOLUTION_8G;
            case k4G:
                return ACCEL_RESOLUTION_4G;
            case k2G:
                return ACCEL_RESOLUTION_2G;

            default:
                throw new IllegalArgumentException("Invalid range given");
        }
    }

    //gets the range requested
    private int getSettingFromRange(Range range) {
        switch (range) {
            case k2G:
                return ACCEL_SETTING_2G;
            case k4G:
                return ACCEL_SETTING_4G;
            case k8G:
                return ACCEL_SETTING_8G;
            case k16G:
                return ACCEL_SETTING_16G;
            default:
                throw new IllegalArgumentException("Invalid range given");
        }
    }


    // MARK - device config methods

    private void setSleepMode(boolean activate) {
        int setting = activate ?
                0b0100_0000  // sleep bit on
                : 0b0000_0000; // otherwise set everything to 0 and enable
        // ref pg 40
        i2c_conn.write(PWR_MGMT_1, setting);
    }

    private void resetDevice() {
        i2c_conn.write(RESET_ADDRESS, RESET_VAL);
    }

    // MARK - device misc methods

    /**
     * This method checks whether the sensor is connected
     * @return  whether or not the sensor is connected
     */
    public boolean isConnected() {
        return i2c_conn.verifySensor(WHO_AM_I, 1, WHO_AM_I_DEFAULT);
    }



    // MARK - Sensor Robustness / Watchdog Methods
    private void reconnectDevice() {
        System.out.println("Resetting " + this.toString());
        this.resetDevice();
        this.watchdog.reset(); // assume that the sensor recovered
    }


    protected void updateWatchdog() {
        // only feed watchdog if the sensor is connected. otherwise don't
        if (this.isConnected() && this.watchdogEnabled) {
            this.watchdog.reset();
        } else if (this.watchdogEnabled) {
            System.err.println("WARNING: Accel GY521 has been disconnected/put into an error state. An attempt to reconnect will be made");
        }
    }

    protected void initWatchdog() {
        if (this.watchdogEnabled) {
            this.watchdog.enable();
        } else {
            this.watchdog.disable();
        }
    }


    // MARK - Accelerometer Methods

    // The reason we cast the word to short is because
    // the MPU6050 returns all values in two's complement.
    // Casting to short tells java to interpret the number as such,
    // giving us the actual value.
    @Override
    public double getX() {
        short rawX = (short) I2CUtils.readWord(i2c_conn, ACCEL_XOUT_H);
        double xVal =  (double) rawX / this.getCurrentResolution();
        return xVal;
    }

    @Override
    public double getY() {
        short rawY = (short) I2CUtils.readWord(i2c_conn, ACCEL_YOUT_H);
        double yVal = (double) rawY / this.getCurrentResolution();
        return yVal;
    }

    @Override
    public double getZ() {
        short rawZ = (short) I2CUtils.readWord(i2c_conn, ACCEL_ZOUT_H);
        double zVal = (double) rawZ / this.getCurrentResolution();
        return zVal;
    }

    @Override
    public void calibrate() {
        this.resetDevice();
    }

    @Override
    public void reset() {
        this.resetDevice();
    }


    // MARK - Complimentary filter methods

    @Override
    public double getAngle() {
        return previousAngle;
    }

    /**
     * This method is responsible for updating the sensor readings,
     * as well as keeping the watchdog fed
     */
    public void update() {
        this.updateWatchdog();

        // This is an implementation of a complementary filter
        // at a high level, it combines the gyro and accelerometer readings to get
        // an accurate reading of the angle
        previousAngle = (previousAngle + this.getRate() * getElapsedTime()) * kGyroInfluence
                + (this.getAccelAngle()) * (1 - kGyroInfluence);
    }

    private double getElapsedTime() {
        double timeElapsed = timer.get();
        timer.reset();
        timer.start();
        return timeElapsed;
    }

    public double getAccelAngle() {
        return Math.toDegrees(Math.atan2(this.getY(), this.getX()));
    }

    @Override
    public double getRate() {
        short val = (short) I2CUtils.readWord(i2c_conn, GYRO_ZOUT_H);
        return -1 * (double) val / GYRO_SCALE_MODIFIER_250DEG;
    }

    public int getGyroConfig() {
        return I2CUtils.readWord(i2c_conn, 0x1B); // GYRO_CFNGI
    }

    public int getAccelConfig() {
        return I2CUtils.readWord(i2c_conn, 0x1c);
    }

    @Override
    public void free() {
        try {
            this.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void close() throws Exception{
        // do nothing
    }

    @Override
    public String toString() {
        return "AccelGY521@" + this.deviceAddr;
    }
}
