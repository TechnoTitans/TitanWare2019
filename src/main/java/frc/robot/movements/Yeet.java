package frc.robot.movements;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.TechnoTitan;

public class Yeet extends TimedCommand {
    public Yeet() {
        super(1.75);
        requires(TechnoTitan.drive);
    }

    @Override
    protected void execute() {
        TechnoTitan.drive.set(0.75);

    }

    @Override
    protected void end() {
        TechnoTitan.drive.stop();
    }
}
