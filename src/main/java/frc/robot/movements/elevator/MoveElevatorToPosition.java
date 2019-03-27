package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.TechnoTitan;

public class MoveElevatorToPosition extends CommandGroup {
    public MoveElevatorToPosition(ElevatorPosition position) {
        addSequential(new MoveElevatorPID(position));
        addSequential(new ConditionalCommand(new MoveElevatorDown()) {
            @Override
            protected boolean condition() {
                return !TechnoTitan.elevator.areSensorsOverridden() && position.getElevatorHeight() == 0;
            }
        });
    }
}
