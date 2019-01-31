package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.OI;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;

public class ControlArm extends Command {
    private Filter elbowFilter, wristFilter;

    public ControlArm() {
        requires(TechnoTitan.arm);
    }

    public void initialize() {
        elbowFilter = new Filter(0.1);
        wristFilter = new Filter(0.1);
    }

    public void execute() {
        elbowFilter.update(TechnoTitan.oi.getElbowMove());
        wristFilter.update(TechnoTitan.oi.getWristMove());
        TechnoTitan.arm.moveElbow(elbowFilter.getValue());
        // TechnoTitan.arm.moveWrist(elbowFilter.getValue());
        if (TechnoTitan.oi.toggleArmUp()) {
            TechnoTitan.arm.toggleUp();
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
