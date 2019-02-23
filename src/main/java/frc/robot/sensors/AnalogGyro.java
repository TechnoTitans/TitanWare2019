//package frc.robot.sensors;
//
//import edu.wpi.first.wpilibj.interfaces.Gyro;
//
//public class AnalogGyro extends TitanGyro {
//
//    private double angleOffset = 0;
//    private static final int defaultChannel = 0;
//
//    public AnalogGyro(Gyro gyroToAccept) {
//        super(gyroToAccept);
//    }
//
//
////    public AnalogGyro() {
////        this(defaultChannel);
////    }
//
////    public AnalogGyro(int channel) {
////        this.m_gyro = new edu.wpi.first.wpilibj.AnalogGyro(channel);
////    }
//
//    @Override
//    public void resetTo(double angle) {
//        this.angleOffset = this.getAngle() - angle;
//    }
//
//    @Override
//    public void calibrate() {
//        m_internalGyro.calibrate();
//    }
//
//    @Override
//    public void reset() {
//        m_internalGyro.reset();
//    }
//
//    @Override
//    public double getAngle() {
//        // m_gyro_angle - angleOffset = angle_rst
//        // m_gyro_angle - angle_rst = angleOffset
//        return m_internalGyro.getAngle() - angleOffset;
//    }
//
//    @Override
//    public double getRate() {
//        return m_internalGyro.getRate();
//    }
//
//    @Override
//    public void free() {
//        try {
//            m_internalGyro.close();
//        } catch (Exception e) {
//            e.printStackTrace();
//        }
//    }
//
//    @Override
//    public void close() throws Exception {
//        m_internalGyro.close();
//    }
//}
