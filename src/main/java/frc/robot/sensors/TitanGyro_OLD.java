//package frc.robot.sensors;
//
//public class TitanGyro {
//
//    private double angleOffset = 0;
//    private TitanGyro m_internalGyro;
//    private static final int defaultChannel = 0;
//
//    public TitanGyro(TitanGyro gyroToUse) {
//        this.m_internalGyro = gyroToUse;
//    }
//
//
//    public void resetTo(double angle) {
//        this.angleOffset = this.getAngle() - angle;
//    }
//
//
//    public void calibrate() {
//        m_internalGyro.calibrate();
//    }
//
//
//    public void reset() {
//        m_internalGyro.reset();
//    }
//
//
//    public double getAngle() {
//        // m_gyro_angle - angleOffset = angle_rst
//        // m_gyro_angle - angle_rst = angleOffset
//        return m_internalGyro.getAngle() - angleOffset;
//    }
//
//
//    public double getRate() {
//        return m_internalGyro.getRate();
//    }
//
//
//    public void free() {
//        try {
//            m_internalGyro.close();
//        } catch (Exception e) {
//            e.printStackTrace();
//        }
//    }
//
//
//    public void close() throws Exception {
//        m_internalGyro.close();
//    }
//}
