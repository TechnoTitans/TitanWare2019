package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;
import frc.robot.sensors.TitanGyro;

public class ForwardAlignLine extends Command {
    private Gyro gyro;

    private final double kP = 0.05;

    public ForwardAlignLine() {
        requires(TechnoTitan.drive);
        gyro = new TitanGyro(TechnoTitan.centralGyro);
    }

    @Override
    public void initialize() {
        gyro.reset();
    }

    @Override
    public void execute() {
        double error = gyro.getAngle();
        final double speed = 0.2;
        TechnoTitan.drive.set(speed - error * kP, speed + error * kP);
    }

    @Override
    protected boolean isFinished() {
        return TechnoTitan.navx.getRawAccelY() <= -0.25;
    }

    public void end() {
        TechnoTitan.drive.stop();
    }
}
