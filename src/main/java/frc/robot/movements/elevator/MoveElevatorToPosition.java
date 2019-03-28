package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.TechnoTitan;

public class MoveElevatorToPosition extends ConditionalCommand {
    private static final double SAFE_ELEVATOR_HEIGHT = 18;
    private static final double SAFE_WRIST_ANGLE = -80;

    private ElevatorPosition position;

    private static class MoveWristSafe extends CommandGroup {
        MoveWristSafe(double wristAngle) {
            addSequential(new Command() {
                @Override
                protected boolean isFinished() {
                    return TechnoTitan.elevator.getHeight() < SAFE_ELEVATOR_HEIGHT;
                }
            });
            addSequential(new MoveWristPID(wristAngle));
        }
    }

    private static class MoveElevatorWithEnd extends CommandGroup {
        public MoveElevatorWithEnd(double elevatorHeight) {
            addSequential(new MoveElevatorPID(elevatorHeight, 1));
            addSequential(new ConditionalCommand(new MoveElevatorDown()) {
                @Override
                protected boolean condition() {
                    return !TechnoTitan.elevator.areSensorsOverridden() && elevatorHeight == 0;
                }
            });
        }
    }

    private static class MoveElevatorToPositionParallel extends CommandGroup {
        MoveElevatorToPositionParallel(ElevatorPosition position) {
            addParallel(new MoveElevatorWithEnd(position.getElevatorHeight()));
            addParallel(new MoveWristPID(position.getWristAngle()));
        }
    }

    private static class MoveElevatorToPositionWithWristDown extends CommandGroup {
        MoveElevatorToPositionWithWristDown(ElevatorPosition position) {
            addParallel(new MoveWristPID(SAFE_WRIST_ANGLE));
            addSequential(new Command() {
                @Override
                protected boolean isFinished() {
                    return TechnoTitan.wrist.getAngle() > SAFE_WRIST_ANGLE - 2;
                }
            });
            addParallel(new MoveElevatorWithEnd(position.getElevatorHeight()));
            addParallel(new MoveWristSafe(position.getWristAngle()));
        }
    }

    public MoveElevatorToPosition(ElevatorPosition position) {
        super(new MoveElevatorToPositionWithWristDown(position), new MoveElevatorToPositionParallel(position));
        this.position = position;
    }


    @Override
    protected boolean condition() {
        double wristAngle = TechnoTitan.wrist.getAngle(),
                elevatorHeight = TechnoTitan.elevator.getHeight();
        return (wristAngle < SAFE_WRIST_ANGLE && position.getWristAngle() < SAFE_WRIST_ANGLE)
                && (elevatorHeight > SAFE_ELEVATOR_HEIGHT && position.getElevatorHeight() < SAFE_ELEVATOR_HEIGHT);
    }
}
