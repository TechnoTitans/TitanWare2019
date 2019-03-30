package frc.robot.movements;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.TechnoTitan;

public class ReleaseHatch extends CommandGroup {
    public ReleaseHatch() {
        requires(TechnoTitan.grabber);
        requires(TechnoTitan.drive);
        addSequential(new InstantCommand(() -> {
            
            TechnoTitan.grabber.setHatchGrab(true);
            //TechnoTitan.drive.set(0.3);
        }));
        addSequential(new WaitCommand(0.1));
        addSequential(new InstantCommand(() -> {
            TechnoTitan.grabber.setExtendHatchMechPiston(false);
            TechnoTitan.drive.set(-0.3);
        }));
        addSequential(new WaitCommand(0.25));
        addSequential(new InstantCommand(() -> {
            TechnoTitan.drive.set(0);
        }));
    
    }

    @Override
    public void interrupted() {
        TechnoTitan.grabber.setExtendHatchMechPiston(false);
        TechnoTitan.drive.set(0);
    }
}
