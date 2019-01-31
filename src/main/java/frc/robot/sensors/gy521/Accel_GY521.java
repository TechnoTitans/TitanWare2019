package frc.robot.sensors.gy521;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

import static java.util.Objects.requireNonNull;
import static frc.robot.sensors.gy521.GY521_Constants.*;

public class Accel_GY521 implements Accelerometer {
    private static I2C accel;
    private double x_accel, y_accel, z_accel;
    private Range currRange;

    //initializes all values
    public Accel_GY521(int address){
        accel = new I2C(I2C.Port.kOnboard, address);
        currRange = Range.k2G;
        this.x_accel = 0;
        this.y_accel = 0;
        this.z_accel = 0;

        this.reset();
    }

    private void reset() {
        accel.write(RESET_ADDRESS, RESET_VAL);
    }

    //checks whether the sensor is connected
    public boolean isConnected(){
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

    private void updateAllValues() {
        double currentResolution = getCurrentResolution();
        ByteBuffer rawBuffer = ByteBuffer.allocate(6);
        accel.read(OUT_X_H, 6, rawBuffer);

        rawBuffer.order(ByteOrder.LITTLE_ENDIAN); // Low, High ordering means that LSB is first, so this is necessary
        System.out.println("The buffer is " + Arrays.toString(rawBuffer.array()));

        //  X_H  X_L  Y_H  Y_L  Z_H  Z_L
        //  0    1    2    3    4    5
        double rawX = rawBuffer.getShort(0); // creates a short from [0,1] (Short.BYTES == 2)
        double rawY = rawBuffer.getShort(2); // creates a short from [2,3]
        double rawZ = rawBuffer.getShort(4);

        this.x_accel = (rawX / currentResolution) /* * STANDARD_GRAVITY*/;
        this.y_accel = (rawY / currentResolution) /* * STANDARD_GRAVITY*/;
        this.z_accel = (rawZ / currentResolution) /* * STANDARD_GRAVITY*/;
    }

    //gets the resolution requestion
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
        this.updateAllValues();
        return x_accel;
    }

    @Override
    public double getY() {
        this.updateAllValues();
        return y_accel;
    }

    @Override
    public double getZ() {
        this.updateAllValues();
        return z_accel;
    }
}
