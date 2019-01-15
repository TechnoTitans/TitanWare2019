package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;

public class ControlArm extends Command {
    private OI oi;
    private Filter elbowFilter, wristFilter;

    private boolean isUp = false;

    public ControlArm() {
        requires(TechnoTitan.arm);
        this.oi = TechnoTitan.oi;
    }

    public void initialize() {
        elbowFilter = new Filter(0.1);
        wristFilter = new Filter(0.1);
    }

    public void execute() {
        elbowFilter.update(oi.getElbowMove());
        wristFilter.update(oi.getWristMove());
        TechnoTitan.arm.moveElbow(elbowFilter.getValue());
        TechnoTitan.arm.moveWrist(elbowFilter.getValue());
        if (oi.toggleArmUp()) {
            isUp = !isUp;
            TechnoTitan.arm.setArmSolenoid(isUp);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
