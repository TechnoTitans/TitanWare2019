package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.TechnoTitan;

public class Launch extends TimedCommand {
    private double speed;

    public Launch(boolean isBeggining) {
        super(isBeggining ? .75 : 1.25);
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
        TechnoTitan.drive.set(speed, speed);
        TechnoTitan.grabber.hold();
    }

    @Override
    protected void end() {
        TechnoTitan.drive.stop();
        TechnoTitan.grabber.stop();
    }

}
