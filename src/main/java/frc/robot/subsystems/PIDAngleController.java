package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDAngleController extends PIDController {
    private final double maxSteadyVoltage;
    /**
     * Same as normal PID controls but takes the maximum voltage (at 0 degrees) to keep away from gravity
     * @param name
     * @param Kp
     * @param Ki
     * @param Kd
     * @param maxSteadyVoltage
     * @param sensor
     * @param output
     */
    public PIDAngleController(String name, double Kp, double Ki, double Kd, double maxSteadyVoltage, PIDSource sensor, PIDOutput output) {
        super(Kp, Ki, Kd, sensor, output);
        setName(name);
        this.maxSteadyVoltage = maxSteadyVoltage;
    }

    public void updateSmartdashboard() {
        String elbowOrWrist = getName();
        SmartDashboard.putNumber("PID " + elbowOrWrist + " error", this.getError());
        SmartDashboard.putNumber("PID " + elbowOrWrist + " output", this.get());
    }

    @Override
    protected double calculateFeedForward() {
        return maxSteadyVoltage * Math.cos(Math.toRadians(m_pidInput.pidGet()));
    }

}
