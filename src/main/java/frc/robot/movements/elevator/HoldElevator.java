package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;

public class HoldElevator extends Command {
    public HoldElevator() {
        requires(TechnoTitan.elevator);
    }

    public void execute() {
        TechnoTitan.elevator.setTargetHeight(TechnoTitan.elevator.getHeight());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
