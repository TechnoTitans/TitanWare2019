package frc.robot.movements;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;

public class ControlArmPID extends Command {

    // Setup
    private static final double kWristP = 0;
    private static final double kWristI = 0;
    private static final double kWristD = 0;

    private static final double kElbowP = 0;
    private static final double kElbowI = 0;
    private static final double kElbowD = 0;


    private PIDController elbowController;
    private PIDController wristController;

    public ControlArmPID(ArmPosition positionInfo) {
        requires(TechnoTitan.arm);

        this.elbowController = new PIDController(kElbowP, kElbowI, kElbowD, TechnoTitan.arm.elbowSensor, output -> TechnoTitan.arm.moveElbow(output));
        this.elbowController.setOutputRange(-1, 1);
        this.elbowController.setPercentTolerance(15); // todo configure tolerance of pid controllers
        this.elbowController.setSetpoint(positionInfo.getElbowAngle());

        // wrist
        this.wristController = new PIDController(kWristP, kWristI, kWristD, TechnoTitan.arm.wristSensor, output -> TechnoTitan.arm.moveWrist(output));
        this.wristController.setOutputRange(-1, 1);
        this.wristController.setPercentTolerance(15);
        this.wristController.setSetpoint(positionInfo.getWristAngle());

    }


    @Override
    protected void initialize() {
        this.elbowController.enable();
        this.wristController.enable();
    }

    @Override
    protected void end() {
        this.elbowController.disable();
        this.wristController.disable();
    }

    @Override
    protected boolean isFinished() {
        return this.elbowController.onTarget() && this.wristController.onTarget();
    }
}
