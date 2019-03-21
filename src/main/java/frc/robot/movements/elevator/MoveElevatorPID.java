package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;

public class MoveElevatorPID extends Command {
    private ElevatorPosition position;

    public MoveElevatorPID(ElevatorPosition position) {
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
        double wristError = Math.abs(TechnoTitan.wrist.getAngle() - position.getWristAngle());
        double elevatorError = Math.abs(TechnoTitan.elevator.getHeight() - position.getElevatorHeight());

        return wristError < 2 && elevatorError < 1;
    }
}