package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;

@SuppressWarnings("WeakerAccess")

public class TitanGyro implements Gyro {

    protected Gyro m_internalGyro;
    private double angleOffset = 0;
    private double scale;

    /**
     * The reason we make a class that accepts a gyro is because the alternative would be to create a central shared
     * gyro in TechnoTitan and reuse that is because there are a lot of issues with sharing such as resetting and shared global
     * state™. And we know that Shared Global State™ is a Very Bad Thing®.
     * @param gyroToAccept - the underlying gyro to use
     */
    public TitanGyro(Gyro gyroToAccept) {
        this(gyroToAccept, 1 / 4.3);
    }

    public TitanGyro(Gyro gyroToAccept, double scale) {
        this.m_internalGyro = gyroToAccept;
        this.scale = scale;
    }

    public void resetTo(double angle) {
        this.angleOffset = m_internalGyro.getAngle() * scale - angle;
    }

    @Override
    public void calibrate() {
        m_internalGyro.calibrate();
    }

    @Override
    public void reset() {
        resetTo(0);
    }

    @Override
    public double getAngle() {
//         m_gyro_angle - angleOffset = angle_rst
//         m_gyro_angle - angle_rst = angleOffset
        return m_internalGyro.getAngle() * scale - angleOffset;
    }

    @Override
    public double getRate() {
        return m_internalGyro.getRate() * scale;
    }

    @Override
    public void free() {
        try {
            m_internalGyro.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void close() throws Exception {
        m_internalGyro.close();
    }
}
