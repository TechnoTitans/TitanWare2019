package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;

public class MoveElevatorToPosition extends Command {
    public MoveElevatorToPosition(ElevatorPosition position) {

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}