package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TechnoTitan;
import frc.robot.sensors.TitanGyro;
import frc.robot.sensors.vision.VisionSensor;

public class AutoAlignAngle extends Command {
    private TitanGyro gyro;
    private int onTargetCount = 0;

    private static final double kP = 0.06;
    private static final double MAX_ERROR = kP * 5;
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
        double error = Math.min(MAX_ERROR, Math.abs(angle) * kP) * Math.signum(angle);
        TechnoTitan.drive.set(-error, error);
    }

    @Override
    protected boolean isFinished() {
        double angle = gyro.getAngle();
        if (Math.abs(angle) < 0.5)
            onTargetCount++;
        else
            onTargetCount = 0;
        return onTargetCount >= 5;
    }

    @Override
    protected void end() {
        TechnoTitan.drive.set(0);
    }
}
