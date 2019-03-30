package frc.robot.movements;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.TechnoTitan;
import frc.robot.movements.elevator.ElevatorPosition;
import frc.robot.movements.elevator.MoveElevatorPID;

public class GrabHatch extends CommandGroup {
    public GrabHatch() {
        addSequential(new InstantCommand(TechnoTitan.grabber, () -> TechnoTitan.grabber.setHatchGrab(false)));
        addSequential(new MoveElevatorPID(ElevatorPosition.LOW_HATCH.getElevatorHeight() + 2, 1));
        addSequential(new InstantCommand(TechnoTitan.drive, () -> TechnoTitan.drive.set(-0.5)));
        addSequential(new WaitCommand(0.2));
        addSequential(new InstantCommand(TechnoTitan.drive, () -> TechnoTitan.drive.stop()));
    }
}
