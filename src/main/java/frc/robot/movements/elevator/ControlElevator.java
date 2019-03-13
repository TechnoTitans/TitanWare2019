package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;

public class ControlElevator extends Command {

    private Filter elevatorFilter, wristFilter;


    private static final double MAX_ELEVATOR_SPEED = 1;
    private static final double MAX_WRIST_SPEED = 0.6;

    public ControlElevator() {
        requires(TechnoTitan.elevator);
    }

    public void initialize() {
        elevatorFilter = new Filter(0.1);
        wristFilter = new Filter(0.1);
    }


    public void execute() {
        elevatorFilter.update(TechnoTitan.oi.getElevatorMove());
        wristFilter.update(TechnoTitan.oi.getWristMove());

        TechnoTitan.elevator.moveElevator(elevatorFilter.getValue() * MAX_ELEVATOR_SPEED);
        TechnoTitan.elevator.moveWrist(wristFilter.getValue() * MAX_WRIST_SPEED);
    }


    @Override
    protected boolean isFinished() {
        return false;
    }
}
