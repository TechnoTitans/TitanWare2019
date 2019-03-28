package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;

public class MoveElevatorPID extends Command {
    private double elevatorHeight, maxError;

    public MoveElevatorPID(double elevatorHeight, double maxError) {
        requires(TechnoTitan.elevator);
        this.elevatorHeight = elevatorHeight;
        this.maxError = maxError;
    }

    public MoveElevatorPID(double elevatorHeight) {
        this(elevatorHeight, -1);
    }

    @Override
    protected void execute() {
        TechnoTitan.elevator.setTargetHeight(elevatorHeight);
    }

    @Override
    protected boolean isFinished() {
        double elevatorError = Math.abs(TechnoTitan.elevator.getHeight() - elevatorHeight);
        return elevatorError < maxError;
    }
}