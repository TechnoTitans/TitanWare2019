package frc.robot.movements.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;

public class ControlArm extends Command {
    private Filter elbowFilter, wristFilter;

    private static final double MAX_ELBOW_SPEED = 0.5;
    private static final double MAX_WRIST_SPEED = 0.5;

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
        TechnoTitan.arm.moveElbow(elbowFilter.getValue() * MAX_ELBOW_SPEED);
         TechnoTitan.arm.moveWrist(wristFilter.getValue() * MAX_WRIST_SPEED);
        if (TechnoTitan.oi.toggleArmUp()) {
            TechnoTitan.arm.toggleUp();
        }
        if (TechnoTitan.oi.toggleOverrideSensors()) {
            TechnoTitan.arm.toggleOverrideSensors();
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
