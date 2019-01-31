package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.TechnoTitan;
import frc.robot.sensors.NavXGyro;

public class AutoAlignStateless extends Command {
    private Gyro gyro;


    private static final double STRAIGHT_END_COEFF = 3,
                                AVOID_TURN_COEFF = 1;

    private static final double ROBOT_RADIUS = 9;

    public AutoAlignStateless() {
        gyro = new NavXGyro();
    }

    @Override
    protected void initialize() {
        gyro.reset();
    }

    /**
     * Calculates the curvature at which to drive
     * @param x  negative when robot is to the left of strips
     * @param y  always negative (since robot is behind strips)
     * @param skew  clockwise is positive
     * @return curvature, positive is move left side faster (i.e. turn rightwards)
     */
    private double calculateCurvature(double x, double y, double skew) {
        /*
        h00(t) = (1+2t)(1-t)^2 = 2t^3 - 3t^2 + 1    p0
        h10(t) = t(1-t)^2 = t^3 - 2t^2 + t          m0
        h01(t) = t^2(3-2t) = -2t^3 + 3t^2           p1
        h11(t) = t^2(t - 1) = t^3 - t^2             m1
         */

        double distance = Math.hypot(x, y);

        // Coefficients of the x polynomial
        // Ax^3 + Bx^2 + Cx + D
        // px = h00 * dx + h10 * mx
        // py = h00 * dy + h10 * my + h11 * distance
        double mx = distance * Math.sin(skew);
        double my = distance * Math.cos(skew);
        double pxB = -3 * x - 2 * mx * AVOID_TURN_COEFF,
                pyB = -3 * y - 2 * my * AVOID_TURN_COEFF - 1 * distance * STRAIGHT_END_COEFF;
        // K = <mx, my> cross <px''(0), py''(0)> / distance^3
        // px''(0) = 2 * pxB
        double kappa = (my * 2 * pxB - mx * 2 * pyB) / Math.pow(distance, 3);

        return kappa;
    }

    @Override
    protected void execute() {
        double dx = TechnoTitan.vision.getXOffset(),
                dy = -TechnoTitan.vision.getYDistance(),
                skew = Math.toRadians(TechnoTitan.vision.getSkew());

        double kappa = calculateCurvature(dx, dy, skew);

        kappa *= ROBOT_RADIUS;
        final double speed = 0.5;

        double lSpeed = speed * (1 + kappa) / 2;
        double rSpeed = speed * (1 - kappa) / 2;

        if (lSpeed > 1 || lSpeed < -1) {
            lSpeed /= Math.abs(lSpeed);
            rSpeed /= Math.abs(lSpeed);
        } else if (rSpeed > 1 | rSpeed < -1) {
            rSpeed /= Math.abs(rSpeed);
            lSpeed /= Math.abs(rSpeed);
        }

        TechnoTitan.drive.set(lSpeed, rSpeed);
    }


    @Override
    protected boolean isFinished() {
        return false;
    }
}
