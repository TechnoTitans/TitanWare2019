package frc.robot.sensors;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

import static java.util.Objects.requireNonNull;
import static frc.robot.sensors.Accelerometer_Constants.*;

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
    }

    private void init() {
        accel.write(RESET_ADDRESS, RESET_VAL); //REVIEW THIS: should reset all the stored values
        //TO DO: look how to initialize everything
    }

    public boolean isConnected(){
        return accel.verifySensor(WHO_AM_I, 1, WHO_AM_I_DEFAULT);
    }

    @Override
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
        return x_accel;
    }

    @Override
    public double getY() {
        return y_accel;
    }

    @Override
    public double getZ() {
        return z_accel;
    }
}
