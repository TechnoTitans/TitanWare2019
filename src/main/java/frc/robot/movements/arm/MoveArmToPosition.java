package frc.robot.movements.arm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.TechnoTitan;

public class MoveArmToPosition extends ConditionalCommand {
    private ArmPosition position;

    private static class MoveArmToPositionSafe extends CommandGroup {
        MoveArmToPositionSafe(ArmPosition position) {
            addSequential(new MoveWristUp());
            addSequential(new MoveArmPiston(position.isSolenoidEnabled()));
            addSequential(new ControlArmPID(position, false, true));
            addSequential(new ControlArmPID(position, true, false));
        }
    }

    private static class MoveArmToPositionFast extends CommandGroup {
        MoveArmToPositionFast(ArmPosition position) {
            addSequential(new MoveArmPiston(position.isSolenoidEnabled()));
            addSequential(new ControlArmPID(position, true, true));
        }
    }

    public MoveArmToPosition(ArmPosition position) {
        super(new MoveArmToPositionSafe(position), new MoveArmToPositionFast(position));
        this.position = position;
    }

    protected boolean condition() {
        double elbowAngle = TechnoTitan.arm.getElbowAngle();
        return elbowAngle > -15 || position.getElbowAngle() > -15;
    }
}
