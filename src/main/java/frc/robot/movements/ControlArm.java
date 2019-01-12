package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.TechnoTitan;

public class ControlArm extends Command {
    private OI oi;

    public ControlArm(OI oi) {
        requires(TechnoTitan.arm);
        this.oi = oi;
    }

    public void execute() {
        TechnoTitan.arm.moveElbow(oi.getElbow());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
