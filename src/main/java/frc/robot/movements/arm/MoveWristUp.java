package frc.robot.movements.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;

public class MoveWristUp extends Command {
    private final double setpoint = 90;

    public MoveWristUp() {
        requires(TechnoTitan.arm);
    }

    @Override
    protected void initialize() {
        TechnoTitan.arm.wristController.setUnfilteredSetpoint(setpoint);
        TechnoTitan.arm.wristController.enable();
        if (!TechnoTitan.arm.elbowController.isEnabled()) {
            TechnoTitan.arm.elbowController.setUnfilteredSetpoint(TechnoTitan.arm.getElbowAngle());
            TechnoTitan.arm.elbowController.enable();  // hold whatever position it had
        }
    }


    @Override
    protected boolean isFinished() {
        return TechnoTitan.arm.wristController.onTarget();
    }
}
