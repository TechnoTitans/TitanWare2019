package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;
import frc.robot.sensors.NavXGyro;

public class ControlDriveTrainStraight extends Command  {
    private Gyro gyro;

    private final double kP = 0.05;

    private Filter filter;

    public ControlDriveTrainStraight() {
        requires(TechnoTitan.drive);
        gyro = new NavXGyro();
        filter = new Filter(0.1);
    }

    @Override
    public void initialize() {
        gyro.reset();
    }

    @Override
    public void execute() {
        double error = gyro.getAngle();
        double speed = (TechnoTitan.oi.getLeft() + TechnoTitan.oi.getRight()) / 2;
        filter.update(speed);
        TechnoTitan.drive.set(filter.getValue() - error * kP, filter.getValue() + error * kP);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
