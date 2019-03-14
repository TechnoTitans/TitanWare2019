package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;

public class MoveElevatorToPosition extends Command {
    private ElevatorPosition position;

    public MoveElevatorToPosition(ElevatorPosition position) {
        requires(TechnoTitan.wrist);
        requires(TechnoTitan.elevator);
        this.position = position;
    }

    @Override
    protected void execute() {
        TechnoTitan.wrist.setTargetAngle(position.getWristAngle());
        TechnoTitan.elevator.setTargetHeight(position.getElevatorHeight());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}