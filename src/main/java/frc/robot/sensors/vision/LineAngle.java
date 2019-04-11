package frc.robot.sensors.vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TechnoTitan;
import frc.robot.sensors.TitanGyro;

public class LineAngle {
    private static final double K_PI = 0; //0.02;

    private Timer timer;

    private double angle;
    private TitanGyro gyro;
    private boolean initialized = false;

    public LineAngle() {
        angle = 0;
        gyro = new TitanGyro(TechnoTitan.centralGyro);
        timer = new Timer();
        timer.start();
    }

    public void update() {
        if (timer.get() > 5) initialized = false;
        double smAngle = SmartDashboard.getNumber("pi-angle", 0);
        if (!initialized) {
//            angle = smAngle;
//            angle = 0;
            angle = -VisionSensor.getAngleTargetDiff();
            initialized = true;
        } else {
            angle += (smAngle - angle) * K_PI + -gyro.getAngle() * (1 - K_PI);
        }
        gyro.reset();
        timer.reset();
    }

    public void reset() {
        initialized = false;
    }

    public double getAngle() {
        return angle;
    }
}
