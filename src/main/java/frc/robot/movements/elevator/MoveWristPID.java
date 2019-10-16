package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TechnoTitan;

public class MoveWristPID extends Command {
    private double wristAngle;
    private double maxError;

    public MoveWristPID(double wristAngle, double maxError) {
        requires(TechnoTitan.wrist);
        this.wristAngle = wristAngle;
        this.maxError = maxError;
    }

    @Override
    protected void execute() {
        // Note: this does not need to be called repeatedly and can be safely set in the initialize method
        // however, we are putting it in execute to be sure
        SmartDashboard.putNumber("The target angle", wristAngle);
        TechnoTitan.wrist.setTargetAngle(wristAngle);
    }

    @Override
    protected boolean isFinished() {
        double wristError = Math.abs(TechnoTitan.wrist.getAngle() - wristAngle);

        return wristError < maxError;
        // return false;
    }
}
