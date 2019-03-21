package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;

public class MoveElevatorDown extends Command {
    private static final double ELEVATOR_SPEED = 0.1;
    public MoveElevatorDown() {
        requires(TechnoTitan.elevator);
    }

    public void execute() {
        TechnoTitan.elevator.moveElevator(-ELEVATOR_SPEED);
    }

    @Override
    protected boolean isFinished() {
        return TechnoTitan.elevator.isAtBottom();
    }

    public void end() {
        TechnoTitan.elevator.moveElevator(0);
    }
}
