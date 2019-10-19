package frc.robot.movements;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.TechnoTitan;
import frc.robot.movements.elevator.ElevatorPosition;
import frc.robot.movements.elevator.MoveElevatorPID;

public class GrabHatch extends CommandGroup {
    public GrabHatch() {
        // todo test this command
        addSequential(new InstantCommand(TechnoTitan.grabber, () -> TechnoTitan.grabber.setBallMode(false)));
        addSequential(new InstantCommand(TechnoTitan.elevator, () -> TechnoTitan.elevator.moveElevator(1)));
        addSequential(new WaitCommand(1.5));
        addSequential(new InstantCommand(TechnoTitan.elevator, () -> TechnoTitan.elevator.moveElevator(0.2)));
    }
}
