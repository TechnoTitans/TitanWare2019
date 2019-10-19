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
            TechnoTitan.grabber.expelBall();
        } else if (TechnoTitan.oi.shouldIntakeGrabber()) {
            TechnoTitan.grabber.intakeBall();
        } else {
            TechnoTitan.grabber.stop();
        }
//
//        if (TechnoTitan.oi.shouldExpelHatch()) {
////            TechnoTitan.grabber.expelHatch();
//            VisionSensor.resetSkew();
//        }
////        TechnoTitan.grabber.updatePistons();
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
