package frc.robot.movements.arm;

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
    private final ArmPosition positionInfo;


    private PIDController elbowController;
    private PIDController wristController;

    public ControlArmPID(ArmPosition positionInfo) {
        requires(TechnoTitan.arm);

         this.elbowController = new PIDController(kElbowP, kElbowI, kElbowD, TechnoTitan.arm.elbowAngleSensor, output -> TechnoTitan.arm.moveElbow(output));
         this.elbowController.setOutputRange(-1, 1);
         this.elbowController.setAbsoluteTolerance(5); // todo configure tolerance of pid controllers
         this.elbowController.setSetpoint(positionInfo.getElbowAngle());

         // wrist
         this.wristController = new PIDController(kWristP, kWristI, kWristD, TechnoTitan.arm.wristAngleSensor, output -> TechnoTitan.arm.moveWrist(output));
         this.wristController.setOutputRange(-1, 1);
         this.wristController.setAbsoluteTolerance(5);
         this.wristController.setSetpoint(positionInfo.getWristAngle());

        this.positionInfo = positionInfo;

    }


    @Override
    protected void initialize() {
        this.elbowController.enable();
        this.wristController.enable();
        TechnoTitan.arm.setArmSolenoid(this.positionInfo.isSolenoidEnabled());
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
