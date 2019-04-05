package frc.robot.sensors.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TechnoTitan;
import frc.robot.sensors.TitanGyro;

public class LineAngle {
    private static final double K_PI = 0.02;

    private double angle;
    private TitanGyro gyro;
    private boolean initialized = false;

    public LineAngle() {
        angle = 0;
        gyro = new TitanGyro(TechnoTitan.centralGyro);
    }

    public void update() {
        double smAngle = SmartDashboard.getNumber("pi-angle", 0);
        if (!initialized) {
            angle = smAngle;
            initialized = true;
        } else {
            angle += (smAngle - angle) * K_PI + -gyro.getAngle() * (1 - K_PI);
        }
        gyro.reset();
    }

    public double getAngle() {
        return angle;
    }
}
