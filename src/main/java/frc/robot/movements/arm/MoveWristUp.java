package frc.robot.movements.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;

public class MoveWristUp extends Command {
    private final double setpoint = 110;

    public MoveWristUp() {
        requires(TechnoTitan.arm);
    }

    @Override
    protected void initialize() {
        if (!isFinished()) {
            TechnoTitan.arm.wristController.setUnfilteredSetpoint(setpoint);
            TechnoTitan.arm.wristController.enable();
        }
        TechnoTitan.arm.elbowController.enable();  // hold whatever position it had
    }


    @Override
    protected boolean isFinished() {
        double angle = TechnoTitan.arm.getWristAngle();
        return angle > setpoint - 5;
    }

    @Override
    protected void end() {
        TechnoTitan.arm.wristController.disable();
    }
}
