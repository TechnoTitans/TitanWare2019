package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;

public class ControlWrist extends Command {
    private static final double MAX_WRIST_SPEED = 0.6;
    private Filter wristFilter;

    public ControlWrist() {
        requires(TechnoTitan.wrist);
    }

    @Override
    protected void initialize() {
        wristFilter = new Filter(0.1);
    }

    @Override
    protected void execute() {
        wristFilter.update(TechnoTitan.oi.getWristMove());
        TechnoTitan.wrist.moveWrist(wristFilter.getValue() * MAX_WRIST_SPEED);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }


}
