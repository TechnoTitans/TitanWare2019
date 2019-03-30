package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.TechnoTitan;

public class MoveElevatorToPosition extends ConditionalCommand {
    private ElevatorPosition position;
    private static final double SAFE_WRIST_ANGLE = -85;

    private static class MoveElevatorToPositionWristDown extends CommandGroup {
        public MoveElevatorToPositionWristDown(ElevatorPosition position) {
            addParallel(new MoveWristPID(SAFE_WRIST_ANGLE + 5, -1));
            addSequential(new Command() {
                @Override
                protected boolean isFinished() {
                    return TechnoTitan.wrist.getAngle() > SAFE_WRIST_ANGLE;
                }
            });
            addSequential(new MoveElevatorPID(position.getElevatorHeight(), 1));
            addSequential(new MoveWristPID(position.getWristAngle(), 2));
        }
    }

    private static final class MoveElevatorToPositionParallel extends CommandGroup {
        public MoveElevatorToPositionParallel(ElevatorPosition position) {
            addParallel(new MoveWristPID(position.getWristAngle(), 2));
            addSequential(new MoveElevatorPID(position.getElevatorHeight(), 1));
            addSequential(new ConditionalCommand(new MoveElevatorDown()) {
                @Override
                protected boolean condition() {
                    return !TechnoTitan.elevator.areSensorsOverridden() && position.getElevatorHeight() == 0;
                }
            });
        }
    }

    public MoveElevatorToPosition(ElevatorPosition position) {
        super(new MoveElevatorToPositionWristDown(position), new MoveElevatorToPositionParallel(position));
        this.position = position;
    }

    @Override
    protected boolean condition() {
        return Math.abs(TechnoTitan.elevator.getHeight() - position.getElevatorHeight()) > 5 &&
                (TechnoTitan.wrist.getAngle() < SAFE_WRIST_ANGLE || position.getWristAngle() < SAFE_WRIST_ANGLE);
    }
}
