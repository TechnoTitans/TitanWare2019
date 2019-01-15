package frc.robot.movements;

import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.TechnoTitan;

public class ControlArmPID extends PIDCommand {



    public ControlArmPID(String name, double p, double i, double d) {
        super(name, p, i, d);
        requires(TechnoTitan.arm);
    }

    @Override
    protected double returnPIDInput() {
        return TechnoTitan.angleSensor.getAngle();
    }

    @Override
    protected void usePIDOutput(double output) {

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
