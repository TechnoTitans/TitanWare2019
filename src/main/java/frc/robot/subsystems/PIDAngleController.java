package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motor.Filter;

public class PIDAngleController extends PIDController {
    private final double maxSteadyVoltage;

    private double unfilteredSetpoint = 0;
    private Filter setpointFilter;

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
        setName("PID " + name);
        this.maxSteadyVoltage = maxSteadyVoltage;
        unfilteredSetpoint = 0;
        setpointFilter = new Filter(0.05);
    }

    public void updateSmartdashboard() {
        String elbowOrWrist = getName();
        SmartDashboard.putNumber(elbowOrWrist + " error", this.getError());
        SmartDashboard.putNumber(elbowOrWrist + " output", this.get());
    }

    public void updateSetpoint() {
        if (this.isEnabled()) {
            setpointFilter.update(unfilteredSetpoint);
            this.setSetpoint(setpointFilter.getValue());
        } else {
            double angle = m_pidInput.pidGet();
            this.setSetpoint(angle);
            setpointFilter.setValue(angle);
        }
    }

    @Override
    protected double calculateFeedForward() {
        return maxSteadyVoltage * Math.cos(Math.toRadians(m_pidInput.pidGet()));
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Unfiltered setpoint", this::getUnfilteredSetpoint, this::setUnfilteredSetpoint);
    }

    public double getUnfilteredSetpoint() {
        return unfilteredSetpoint;
    }

    public void setUnfilteredSetpoint(double unfilteredSetpoint) {
        this.unfilteredSetpoint = unfilteredSetpoint;
    }
}
