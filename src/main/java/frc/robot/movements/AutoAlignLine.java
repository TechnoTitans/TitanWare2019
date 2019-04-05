package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;
import frc.robot.sensors.vision.LineAngle;

public class AutoAlignLine extends Command {
    private Filter xFilter;
    private LineAngle angle;

    private static final double K_ANGLE = 0.01, K_X = 0.001;
    private final double speed = 0.3;

    public AutoAlignLine() {
        requires(TechnoTitan.drive);
        angle = new LineAngle();
        xFilter = new Filter(0.1);
    }

    @Override
    protected void execute() {
        angle.update();
        xFilter.update(SmartDashboard.getNumber("pi-x-offset", 0));

        double error = K_ANGLE * angle.getAngle() + K_X * xFilter.getValue();
        TechnoTitan.drive.set(speed + error, speed - error);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    public void end() {
        TechnoTitan.drive.stop();
    }
}
