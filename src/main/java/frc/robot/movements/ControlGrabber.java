package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;

public class ControlGrabber extends Command {
    public ControlGrabber() {
        requires(TechnoTitan.grabber);
    }

    @Override
    protected void execute() {
        if (TechnoTitan.oi.shouldExpelGrabber()) {
            TechnoTitan.grabber.expel();
        } else if (TechnoTitan.oi.shouldIntakeGrabber()) {
            TechnoTitan.grabber.intake();
        } else {
            TechnoTitan.grabber.stop();
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        TechnoTitan.grabber.stop();
    }
}
