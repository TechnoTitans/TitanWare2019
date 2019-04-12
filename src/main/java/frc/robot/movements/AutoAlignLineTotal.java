package frc.robot.movements;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoAlignLineTotal extends CommandGroup {
    public AutoAlignLineTotal() {
        addSequential(new AutoAlignAngle(), 1);
        addSequential(new ForwardAlignLine(), 1);
    }
}
