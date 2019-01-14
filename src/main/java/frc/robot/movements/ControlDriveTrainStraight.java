package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.OI;
import frc.robot.TechnoTitan;
import frc.robot.sensors.NavXGyro;

public class ControlDriveTrainStraight extends Command  {
    private OI oi;
    private Gyro gyro;

    private final double kP = 0.05;

    public ControlDriveTrainStraight(OI oi) {
        requires(TechnoTitan.drive);
        this.oi = oi;
        gyro = new NavXGyro();
    }

    @Override
    public void initialize() {
        gyro.reset();
    }

    @Override
    public void execute() {
        double error = gyro.getAngle();
        double speed = (oi.getLeft() + oi.getRight()) / 2;
        TechnoTitan.drive.set(speed - error * kP, speed + error * kP);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
