package frc.robot.sensors;

import edu.wpi.first.wpilibj.interfaces.Gyro;

@SuppressWarnings("WeakerAccess")
public class TitanGyro implements Gyro {

    protected Gyro m_internalGyro;
    private double angleOffset = 0;

    public TitanGyro(Gyro gyroToAccept) {
        this.m_internalGyro = gyroToAccept;
    }

    public void resetTo(double angle) {
        this.angleOffset = this.getAngle() - angle;
    }

    @Override
    public void calibrate() {
        m_internalGyro.calibrate();
    }

    @Override
    public void reset() {
        m_internalGyro.reset();
    }

    @Override
    public double getAngle() {
//         m_gyro_angle - angleOffset = angle_rst
//         m_gyro_angle - angle_rst = angleOffset
        return m_internalGyro.getAngle() - angleOffset;
    }

    @Override
    public double getRate() {
        return m_internalGyro.getRate();
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
