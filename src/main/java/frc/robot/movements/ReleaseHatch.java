package frc.robot.movements;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.TechnoTitan;

public class ReleaseHatch extends CommandGroup {
    public ReleaseHatch() {
        requires(TechnoTitan.grabber);
        addSequential(new InstantCommand(() -> {
            TechnoTitan.grabber.setClawPistons(false); // make sure that they are in
            TechnoTitan.grabber.setPancakePistons(true);
        }));
        addSequential(new WaitCommand(0.25));
        addSequential(new InstantCommand(() -> {
            TechnoTitan.grabber.setPancakePistons(false);
        }));
    }

    @Override
    public void interrupted() {
        TechnoTitan.grabber.setPancakePistons(false);
    }
}
