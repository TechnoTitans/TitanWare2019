package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TechnoTitan;

public class MoveWristPID extends Command {
    private static final double MIN_VALID_ANGLE = -120;
    private static final double MAX_VALID_ANGLE = 100;
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
        double currentAngle = TechnoTitan.wrist.getAngle();
        double wristError = Math.abs(currentAngle - wristAngle);

        // Emergency Protection
        // If the current angle of the wrist isn't between the defined min and max angles, we will stop immediately
        if (!(MIN_VALID_ANGLE < currentAngle && currentAngle < MAX_VALID_ANGLE)) {
            return true;
        } else {
            return wristError < maxError;
        }
    }
}
