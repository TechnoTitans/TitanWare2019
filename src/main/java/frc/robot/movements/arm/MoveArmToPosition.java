package frc.robot.movements.arm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.TechnoTitan;

public class MoveArmToPosition extends ConditionalCommand {
    private ArmPosition position;

    private static void disableNecessaryPIDs(ArmPosition position, CommandGroup command) {
        if (position == ArmPosition.STOW_POSITION) {
            // disable pid
            command.addSequential(new InstantCommand(() -> TechnoTitan.arm.elbowController.disable()));
        }
        if (position == ArmPosition.HATCH_PICKUP || position == ArmPosition.STOW_POSITION) {
            command.addSequential(new InstantCommand(() -> TechnoTitan.arm.wristController.reset()));
        }
    }

    private static class MoveArmToPositionSafe extends CommandGroup {
        MoveArmToPositionSafe(ArmPosition position) {
            addSequential(new MoveWristUp());
            addSequential(new MoveArmPiston(position.isSolenoidEnabled()));
//            addSequential(new ConditionalCommand((new InstantCommand(() -> TechnoTitan.arm.wristController.disable()))) {
//                @Override
//                protected boolean condition() {
//                    return position.getElbowAngle() < TechnoTitan.arm.getElbowAngle();
//                }
//            });
            addSequential(new ControlArmPID(position, false, true));
            addSequential(new ControlArmPID(position, true, false));
            disableNecessaryPIDs(position, this);
        }
    }

    private static class MoveArmToPositionFast extends CommandGroup {
        MoveArmToPositionFast(ArmPosition position) {
            addSequential(new MoveArmPiston(position.isSolenoidEnabled()));
            addSequential(new ControlArmPID(position, true, true));
            disableNecessaryPIDs(position, this);
        }
    }

    public MoveArmToPosition(ArmPosition position) {
        super(new MoveArmToPositionSafe(position), new MoveArmToPositionFast(position));
        requires(TechnoTitan.arm);
        this.position = position;
    }

    protected boolean condition() {
        double elbowAngle = TechnoTitan.arm.getElbowAngle();
        return elbowAngle > -5 || position.getElbowAngle() > -5;
    }
}
