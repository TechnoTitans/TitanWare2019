package frc.robot.sensors.util;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sensors.lis3dh.Accel_LIS3DH;

public class AngleCalculator implements PIDSource {
    private PIDSourceType pidSourceType = PIDSourceType.kDisplacement;

    // Note: all angles are expressed in RADIANS, so that the multiplicative factor of pi goes away
    private double angle = 0; // start at 0 rad
    private double omega = 0; // change in angle, in rad/s

    private double UNITS_PER_FT = 1;

    private static final double GRAVITY = 32.17;  // ft/s^2

    private Accel_LIS3DH accel;
    private double radius;

    private double time = 0;
    private Timer timer;

    public AngleCalculator(Accel_LIS3DH accel, double radius) {
        this.accel = accel;
        this.radius = radius;
        timer = new Timer();
        timer.start();
    }

    public double getAngle() {
        return Math.toDegrees(angle);
    }

    public double getRate() {
        return Math.toDegrees(omega);
    }

    /**
     * Calibrate, this sets the value of gravity initializes the angle.
     * It must only be called when the sensor is still (e.g. beginning of the match)
     */
    public void calibrate() {
        double ax = accel.getX();
        double ay = accel.getY();
        UNITS_PER_FT = Math.hypot(ax, ay) / GRAVITY;
        angle = Math.atan2(ay, ax);
        time = timer.get();
    }

    // update angle and omega with the acceleration values
    public void update() {
        // convert accelerations into ft/s^2
        // Assumptions about sign: when the arm is horizontal, +x is down and +y is inwards (i.e. towards robot)
        double ax = accel.getX() / UNITS_PER_FT;
        double ay = accel.getY() / UNITS_PER_FT;
        double dt = timer.get() - time;
        // BEGIN MATH
        // Let alpha be acceleration in rad/s^2
        // Solve:
        // ax = g*cos(angle + omega * dt + 0.5 * alpha * dt^2) + alpha*R
        // When dt is near 0, ax = g*cos(angle) - g*sin(angle)*(omega*dt + 0.5*alpha*dt^2) + alpha*R
        // ax = g*(cos(angle)  - sin(angle)*omega*dt) + alpha*(R - 0.5*g*sin(angle)*dt^2)
        // Similarly, ay = g*sin(angle + omega*dt + 0.5 * alpha * dt^2) - R*(omega+alpha*dt)^2
        // ay = g*sin(angle) + g*cos(angle)*(omega*dt + 0.5*alpha*dt^2) - R*omega^2 - 2R*omega*alpha*dt
        // ay = g*(sin(angle)+cos(angle)*omega*dt) - R*omega^2 + alpha*(0.5*g*cos(angle)*dt^2 - 2R*omega*dt)
        // let ax = bx + mx * alpha and ay = by + my * alpha
        // Find the alpha such that the distance from the points (ax, ay) and the true reading is minimized
        // To do this, we use the vector projection formula, proj_v(u) = u*v/(v*v) * v
        // END MATH
        double bx = GRAVITY * (Math.cos(angle) - Math.sin(angle) * omega * dt);
        double mx = radius - 0.5 * GRAVITY * dt * dt * Math.sin(angle);

        double by = GRAVITY * (Math.sin(angle) + Math.cos(angle)*omega*dt) - radius*omega*omega;
        double my = 0.5 * GRAVITY * dt * dt * Math.cos(angle) - 2 * radius * omega * dt;

        double alpha = (mx * (ax - bx) + my * (ay - by)) / (mx * mx + my * my);

        angle += omega * dt + 0.5 * alpha * dt * dt;
        omega += alpha * dt;

        time = timer.get();
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        this.pidSourceType = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return pidSourceType;
    }

    @Override
    public double pidGet() {
        switch (pidSourceType) {
            case kDisplacement:
                return getAngle();
            case kRate:
                return getRate();
            default:
                throw new IllegalStateException("Invalid pidsourcetype at this point in execution");
        }
    }
}
