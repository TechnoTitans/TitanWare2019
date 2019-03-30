package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;

public class MoveElevatorPID extends Command {
    private double elevatorHeight, maxError;
    private static final double MIN_HEIGHT = 3;

    public MoveElevatorPID(double elevatorHeight, double maxError) {
        requires(TechnoTitan.elevator);
        this.elevatorHeight = Math.max(elevatorHeight, MIN_HEIGHT);
        this.maxError = maxError;
    }

    @Override
    protected void execute() {
        // Note: this does not need to be called repeatedly and can be safely set in the initialize method
        // however, we are putting it in execute to be sure
        TechnoTitan.elevator.setTargetHeight(elevatorHeight);
    }

    @Override
    protected boolean isFinished() {
        double elevatorError = Math.abs(TechnoTitan.elevator.getHeight() - elevatorHeight);
        return elevatorError < maxError;
    }
}