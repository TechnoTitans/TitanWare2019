package frc.robot.movements;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.TechnoTitan;

public class Launch extends TimedCommand {
    private double speed;
    private static final double CUTOFF_DRIVE_TRAIN = 0.75;

    public Launch(boolean isBeggining) {
        super(isBeggining ? 1.25 : 1.75);
        speed = isBeggining ? 0.8 : 1;
        requires(TechnoTitan.drive);
        requires(TechnoTitan.grabber);
    }

    @Override
    protected void initialize() {
//        TechnoTitan.arm.wristController.setUnfilteredSetpoint(88);
//        TechnoTitan.arm.wristController.enable();
    }

    @Override
    protected void execute() {
        if (timeSinceInitialized() <= CUTOFF_DRIVE_TRAIN) TechnoTitan.drive.set(speed, speed);
        else TechnoTitan.drive.stop();
        TechnoTitan.grabber.holdBall();

    }

    @Override
    protected void end() {
        TechnoTitan.drive.stop();
        TechnoTitan.grabber.stop();
    }

}
