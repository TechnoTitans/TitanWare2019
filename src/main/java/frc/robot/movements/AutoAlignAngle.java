package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;
import frc.robot.sensors.TitanGyro;
import frc.robot.sensors.vision.VisionSensor;

public class AutoAlignAngle extends Command {
    private TitanGyro gyro;

    public AutoAlignAngle() {
        requires(TechnoTitan.drive);
        gyro = new TitanGyro(TechnoTitan.centralGyro);
    }

    @Override
    protected void initialize() {
        gyro.resetTo(VisionSensor.getAngleTargetDiff());
    }

    @Override
    protected void execute() {
        double angle = gyro.getAngle();
        TechnoTitan.drive.set(-angle * 0.05, angle * 0.05);
    }

    @Override
    protected boolean isFinished() {
        double angle = gyro.getAngle();
        return Math.abs(angle) < 1;
    }

    @Override
    protected void end() {
        TechnoTitan.drive.set(0);
    }
}
