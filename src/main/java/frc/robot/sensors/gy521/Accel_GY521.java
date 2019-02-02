package frc.robot.sensors.gy521;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import java.nio.ByteBuffer;

import static frc.robot.sensors.gy521.GY521_Constants.*;
import static java.util.Objects.requireNonNull;

@SuppressWarnings({"Duplicates", "UnnecessaryLocalVariable", "SameParameterValue"})
public class Accel_GY521 implements Accelerometer, Gyro {

    // MARK - complimentary config
    private static final double kGyroInfluence = 0.98;



    private static I2C accel;
    private Range currRange;
    private double previousAngle = 0.0;
    private Timer time;

    // TODO get link to manual
    // TODO Create an init sen

    //initializes all values
    public Accel_GY521(int address) {
        accel = new I2C(I2C.Port.kOnboard, address);
        currRange = Range.k2G;
        this.time = new Timer();
        this.resetDevice();
        this.setSleepMode(false);
    }

    private void setSleepMode(boolean activate) {
        int setting = activate ?
                0b0100_0000  // sleep bit on
              : 0b0000_0000; // otherwise set everything to 0 and enable
            // ref pg 40
        accel.write(PWR_MGMT_1, setting);
    }

    private void resetDevice() {
        accel.write(RESET_ADDRESS, RESET_VAL);
    }

    //checks whether the sensor is connected
    public boolean isConnected() {
        return accel.verifySensor(WHO_AM_I, 1, WHO_AM_I_DEFAULT);
    }

    @Override
    //sets the range for the accelerometer
    public void setRange(Range range) {
        //Ensures that the range is not null pointer
        requireNonNull(range, "The range that has been entered is invalid");

        ByteBuffer ctrl4Buffer = ByteBuffer.allocate(1);
        accel.read(ACCEL_RANGE, 1, ctrl4Buffer);

        byte ctrl4 = ctrl4Buffer.get();
        ctrl4 &= ~0x18; // this basically resets only the FS1 and FS0 bits
        ctrl4 |= getSettingFromRange(range) << 3; // this sets only the FS1 and FS0

        accel.write(ACCEL_RANGE, ctrl4);

        this.currRange = range;
    }

    private int readWord(int address) {
        // todo check endianness/order of reading
//        ByteBuffer rawBuffer = ByteBuffer.allocate(2);
        // TODO Refactor this
        byte[] highByte = new byte[1];
        byte[] lowByte = new byte[1];
        accel.read(address, 1, highByte);
        accel.read(address + 1, 1, lowByte);

//        rawBuffer.order(ByteOrder.BIG_ENDIAN); // High, Low ordering means that MSB is first
//        System.out.println("The buffer is " + Arrays.toString(rawBuffer.array()));

        int high = highByte[0] & 0xFF; // (<byte> & 0xFF) converts a signed byte into an unsigned byte
        int low = lowByte[0] & 0xFF;
        int value = (high << 8) + low; // todo explain this

        return value;
        //  X_H  X_L  Y_H  Y_L  Z_H  Z_L
        //  0    1    2    3    4    5
//        double rawX = rawBuffer.getShort(0); // creates a short from [0,1] (Short.BYTES == 2)
//        double rawY = rawBuffer.getShort(2); // creates a short from [2,3]
//        double rawZ = rawBuffer.getShort(4);

//        this.x_accel = (rawX / currentResolution) /* * STANDARD_GRAVITY*/;
//        this.y_accel = (rawY / currentResolution) /* * STANDARD_GRAVITY*/;
//        this.z_accel = (rawZ / currentResolution) /* * STANDARD_GRAVITY*/;
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

    @Override
    public double getX() {
        short rawX = (short) this.readWord(ACCEL_XOUT_H);
        double xVal =  (double) rawX / this.getCurrentResolution();
        return xVal;
    }

    @Override
    public double getY() {
        short rawY = (short) this.readWord(ACCEL_YOUT_H);
        double yVal = (double) rawY / this.getCurrentResolution();
        return yVal;
    }

    @Override
    public double getZ() {
        short rawZ = (short) this.readWord(ACCEL_ZOUT_H);
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

    // Complimentary
    @Override
    public double getAngle() {
        return previousAngle;
    }

    public void update() {
        // todo add math and stuff
        // todo explain this magic
        previousAngle = (previousAngle + this.getRate() * getElapsedTime()) * kGyroInfluence
                + (this.getAccelAngle()) * (1 - kGyroInfluence);
    }

    private double getElapsedTime() {
        double timeElapsed = time.get();
        time.reset();
        time.start();
        return timeElapsed;
    }

    public double getAccelAngle() {
        return Math.toDegrees(Math.atan2(this.getY(), this.getX()));
    }

    @Override
    public double getRate() {
        short val = (short) readWord(GYRO_ZOUT_H);
        return -1 * (double) val / GYRO_SCALE_MODIFIER_250DEG;
    }

    public int getGyroConfig() {
        return readWord(0x1B); // GYRO_CFNGI
    }

    public int getAccelConfig() {
        return readWord(0x1c);
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
    public void close() throws Exception {
        // do nothing
    }
}
