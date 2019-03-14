package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;

public class ControlElevator extends Command {

    private Filter elevatorFilter;


    private static final double MAX_ELEVATOR_SPEED = 1;

    public ControlElevator() {
        requires(TechnoTitan.elevator);
    }

    public void initialize() {
        elevatorFilter = new Filter(0.1);
    }


    public void execute() {
        elevatorFilter.update(TechnoTitan.oi.getElevatorMove());
        TechnoTitan.elevator.moveElevator(elevatorFilter.getValue() * MAX_ELEVATOR_SPEED);

        if (TechnoTitan.oi.shouldToggleOverrideSensors()) TechnoTitan.elevator.toggleOverrideSensors();;
    }


    @Override
    protected boolean isFinished() {
        return false;
    }
}
