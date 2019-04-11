package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;
import frc.robot.sensors.vision.LineAngle;

public class AutoAlignLine extends Command {
    private Filter xFilter;

    private static final LineAngle angle = new LineAngle();

    private static final double K_ANGLE = 0.005, K_X = 0.0025;
    private final double speed = 0.15;

    public AutoAlignLine() {
        requires(TechnoTitan.drive);
        xFilter = new Filter(0.1);
    }

    @Override
    protected void execute() {
        angle.update();
        xFilter.update(SmartDashboard.getNumber("pi-x-offset", 0));

        SmartDashboard.putNumber("Auto align angle", angle.getAngle());

        double error = K_ANGLE * angle.getAngle() + K_X * xFilter.getValue();
        TechnoTitan.drive.set(speed + error, speed - error);
    }

    public static void resetLineAngle() {
        angle.reset();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    public void end() {
        TechnoTitan.drive.stop();
    }
}
