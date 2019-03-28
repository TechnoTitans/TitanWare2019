package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;

public class MoveWristPID extends Command {
    private double wristAngle, maxError;

    public MoveWristPID(double wristAngle, double maxError) {
        requires(TechnoTitan.wrist);
        this.wristAngle = wristAngle;
        this.maxError = maxError;
    }

    // Never finishes
    public MoveWristPID(double wristAngle) {
        this(wristAngle, -1);
    }

    @Override
    protected void execute() {
        TechnoTitan.wrist.setTargetAngle(wristAngle);
    }

    @Override
    protected boolean isFinished() {
        double wristError = Math.abs(TechnoTitan.wrist.getAngle() - wristAngle);
        return wristError < maxError;
    }
}
