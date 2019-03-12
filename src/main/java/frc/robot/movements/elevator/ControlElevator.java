package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;

public class ControlElevator extends Command {

    private Filter elbowFilter, wristFilter;
    private static final double MAX_SPEED = 1.0;

    public ControlElevator() {
        requires(TechnoTitan.elevator);
    }


    @Override
    protected boolean isFinished() {
        return false;
    }
}
