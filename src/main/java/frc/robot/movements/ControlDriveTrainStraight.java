package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.OI;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;
import frc.robot.sensors.NavXGyro;

public class ControlDriveTrainStraight extends Command  {
    private OI oi;
    private Gyro gyro;

    private final double kP = 0.05;

    private Filter filter;

    public ControlDriveTrainStraight() {
        requires(TechnoTitan.drive);
        this.oi = TechnoTitan.oi;
        gyro = new NavXGyro();
    }

    @Override
    public void initialize() {
        gyro.reset();
        filter = new Filter(0.1);
    }

    @Override
    public void execute() {
        double error = gyro.getAngle();
        double speed = (oi.getLeft() + oi.getRight()) / 2;
        filter.update(speed);
        TechnoTitan.drive.set(filter.getValue() - error * kP, filter.getValue() + error * kP);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
