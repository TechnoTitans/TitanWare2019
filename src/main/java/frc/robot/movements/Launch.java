package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.TechnoTitan;

public class Launch extends TimedCommand {
    public Launch() {
        super(1.5);
        requires(TechnoTitan.drive);
    }

    @Override
    protected void initialize() {
        TechnoTitan.arm.wristController.setUnfilteredSetpoint(88);
        TechnoTitan.arm.wristController.enable();
    }

    @Override
    protected void execute() {
        TechnoTitan.drive.set(0.5, 0.5);
    }

    @Override
    protected void end() {
        TechnoTitan.drive.stop();
    }

}
