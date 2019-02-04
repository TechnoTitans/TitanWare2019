package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;
import frc.robot.sensors.NavXGyro;

public class AutoAlign extends Command {
    private NavXGyro gyro;

    private final double minSpeed = 0.2;
    private double slowDownDist;

    private static final double STRAIGHT_END_COEFF = 2.5;

    private static final double ROBOT_RADIUS = 15;  // TODO: measure

    private static final double TARGET_Y_OFFSET = 20,
                                NO_SENSOR_DIST = 10;

    private double speed;

    private double lDist, rDist;

    public AutoAlign(double speed, double slowDownDist) {
        gyro = new NavXGyro();
        this.speed = speed;
        this.slowDownDist = slowDownDist;
    }

    @Override
    protected void initialize() {
        gyro.reset();
        if (TechnoTitan.vision.canSeeTargets()) {
            TechnoTitan.visionKalmanFilter.start();
        } else {
            if (this.getGroup() != null) this.getGroup().cancel();
            else this.cancel();
        }
    }

    /**
     * Calculates the curvature at which to drive
     * @param x  negative when robot is to the left of strips
     * @param y  always negative (since robot is behind strips)
     * @param skew  clockwise is positive, in radians (not degrees!)
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

        /*
        A brief explanation of the math:
        We are using a hermite cubic spline to connect the points (x, y) and (0, 0)
        The formula is h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1
        Here, p0, m0, p1, and m1 are vectors representing
          - The initial location (x, y)
          - The initial direction (mx, my)
          - The final location (0, 0)
          - The final direction (0, -distance) * STRAIGHT_END_COEFF
         The lengths of m0 and m1 control how strongly the tangent lines for the initial curves are pulled in
         that direction. By default, when STRAIGHT_END_COEFF is 1, both have the same length as the distance left
         to the target.

         Then, to calculate curvature, we use the formula
         K = (r'(t) x r''(t)) / |r'(t)|^3      (where x denotes the cross product)
         where K is the curvature (the reciprocal of the radius of a circle touching the path)
         This formula is derived from a = v^2 / r = v^2 * K where a is centripetal acceleration
         Then, K = a / v^2, note that r'(t), the velocity, cross r''(t), the acceleration, is a * v
         so K = |r'(t) x r''(t)| / |r'(t)|^3

         Now, by construction, r'(t) = <mx, my> so we only need to find r''(t), for which we only need the
         t^2 coefficient of the hermite polynomials.

         Check out this desmos I made to play with the variables: https://www.desmos.com/calculator/lupevjbr8s
         */

        // px = h00 * x + h10 * mx
        // py = h00 * y + h10 * my + h11 * distance
        double mx = distance * Math.sin(skew);
        double my = distance * Math.cos(skew);
        double pxB = -3 * x - 2 * mx,
                pyB = -3 * y - 2 * my - 1 * distance * STRAIGHT_END_COEFF;
        // K = <mx, my> cross <px''(0), py''(0)> / distance^3
        // px''(0) = 2 * pxB
        double kappa = (my * 2 * pxB - mx * 2 * pyB) / Math.pow(distance, 3);

        return kappa;
    }

    @Override
    protected void execute() {
        double lSpeed, rSpeed;

        if (-TechnoTitan.visionKalmanFilter.getY() > TARGET_Y_OFFSET + NO_SENSOR_DIST) {
            TechnoTitan.visionKalmanFilter.update();

            double dx = TechnoTitan.visionKalmanFilter.getX(),
                    dy = TechnoTitan.visionKalmanFilter.getY(),
                    skew = TechnoTitan.visionKalmanFilter.getAngle();

            double kappa = calculateCurvature(dx, dy + TARGET_Y_OFFSET, skew);

            kappa *= ROBOT_RADIUS;

            double speed = (this.speed - minSpeed) * Math.min(1, (-dy - TARGET_Y_OFFSET - NO_SENSOR_DIST) / slowDownDist) + minSpeed;

            lSpeed = speed * (1 + kappa) / 2;
            rSpeed = speed * (1 - kappa) / 2;

            if (lSpeed > 1 || lSpeed < -1) {
                lSpeed /= Math.abs(lSpeed);
                rSpeed /= Math.abs(lSpeed);
            }
            if (rSpeed > 1 | rSpeed < -1) {
                rSpeed /= Math.abs(rSpeed);
                lSpeed /= Math.abs(rSpeed);
            }
            gyro.resetTo(Math.toDegrees(skew));
            lDist = TechnoTitan.drive.getLeftEncoder().getDistance();
            rDist = TechnoTitan.drive.getRightEncoder().getDistance();
        } else {
            double error = gyro.getAngle() * 0.05;
            lSpeed = minSpeed - error;
            rSpeed = minSpeed + error;
        }
        TechnoTitan.drive.set(lSpeed, rSpeed);
    }

    @Override
    protected boolean isFinished() {
        if (-TechnoTitan.visionKalmanFilter.getY() > TARGET_Y_OFFSET + NO_SENSOR_DIST) return false;
        double dist = (TechnoTitan.drive.getLeftEncoder().getDistance() - lDist) / 2 + (TechnoTitan.drive.getRightEncoder().getDistance() - rDist) / 2;
        return dist >= NO_SENSOR_DIST;
    }

    @Override
    protected void end() {
        TechnoTitan.drive.stop();
    }
}
