package frc.robot.movements.arm;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;

public class ControlArmPID extends Command {

    // Setup
    private static final double kWristP = 0.02;
    private static final double kWristI = 0.004;
    private static final double kWristD = 0;

    private static final double kElbowP = 0.05;
    private static final double kElbowI = 0.005;
    private static final double kElbowD = 0;
    private final ArmPosition positionInfo;


    private PIDController elbowController;
    private PIDController wristController;

    private Filter elbowFilter, wristFilter;

    private boolean moveWrist, moveElbow;

    public ControlArmPID(ArmPosition positionInfo, boolean moveWrist, boolean moveElbow) {
        requires(TechnoTitan.arm);
        this.moveWrist = moveWrist;
        this.moveElbow = moveElbow;
        /*
        elbowFilter = new Filter(0.5);
        wristFilter = new Filter(0.5);

         this.elbowController = new PIDController(kElbowP, kElbowI, kElbowD, TechnoTitan.arm.elbowAngleSensor, output -> {
             elbowFilter.update(output);
             SmartDashboard.putNumber("PID Elbow output", elbowFilter.getValue());
             SmartDashboard.putNumber("PID Elbow Error", elbowController.getError());
             TechnoTitan.arm.moveElbow(elbowFilter.getValue());
         });
         this.elbowController.setOutputRange(-0.5, 0.5);
//         this.elbowController.setAbsoluteTolerance(1); // todo configure tolerance of pid controllers
         this.elbowController.setSetpoint(positionInfo.getElbowAngle());

         // wrist
         this.wristController = new PIDController(kWristP, kWristI, kWristD, TechnoTitan.arm.wristAngleSensor, output -> {
             wristFilter.update(output);
             SmartDashboard.putNumber("PID Wrist output", wristFilter.getValue());
             SmartDashboard.putNumber("PID Wrist Error", wristController.getError());
             TechnoTitan.arm.moveWrist(wristFilter.getValue());
         });
         this.wristController.setOutputRange(-0.5, 0.5);
//         this.wristController.setAbsoluteTolerance(1);
         this.wristController.setSetpoint(positionInfo.getWristAngle());*/

        this.positionInfo = positionInfo;

//        SmartDashboard.putData("Elbow at " + positionInfo, elbowController);
//        SmartDashboard.putData("Wrist at " + positionInfo, wristController);
    }


    @Override
    protected void initialize() {
//        if (TechnoTitan.arm.areSensorsOverriden()) return;
        if (moveElbow) {
            TechnoTitan.arm.elbowController.setUnfilteredSetpoint(positionInfo.getElbowAngle());
            TechnoTitan.arm.elbowController.enable();
        }
        if (moveWrist) {
            TechnoTitan.arm.wristController.setUnfilteredSetpoint(positionInfo.getWristAngle());
            TechnoTitan.arm.wristController.enable();
        }
//        TechnoTitan.arm.setArmSolenoid(this.positionInfo.isSolenoidEnabled());
    }

    @Override
    protected void end() {
//        if (moveElbow) TechnoTitan.arm.elbowController.disable();
//        if (moveWrist) TechnoTitan.arm.wristController.disable();
    }

    @Override
    protected boolean isFinished() {
//        return TechnoTitan.arm.areSensorsOverriden() || (this.elbowController.onTarget() && this.wristController.onTarget());
//        return TechnoTitan.arm.areSensorsOverriden();
        boolean result = true;
        if (moveElbow) result = result && TechnoTitan.arm.elbowController.onTarget();
        if (moveWrist) result = result && TechnoTitan.arm.wristController.onTarget();
        return result;
    }
}
