package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.Motor;
import frc.robot.motor.TalonSRX;
import frc.robot.movements.ControlGrabber;

public class Grabber extends Subsystem {

    private Motor grabberMotorLeft, grabberMotorRight;

    private static final double EXPEL_BALL_SPEED = 1;
    private static final double EXPEL_HATCH_SPEED = -1;

//    private static final double BALL_INTAKE_SPEED = -1;
//    private static final double HATCH_INTAKE_SPEED = 1;
//    private static final double BALL_INTAKE_SPEED = 0.4;
//    private static final double HATCH_INTAKE_SPEED = 0.4;
    private static final double BALL_INTAKE_SPEED = -0.25;
    private static final double HATCH_INTAKE_SPEED = 1;

    private Solenoid ballHatchModePiston;

    public Grabber(TalonSRX grabberMotorLeft, TalonSRX grabberMotorRight, Solenoid ballHatchModePiston) {
        this.grabberMotorLeft = grabberMotorLeft;
        this.grabberMotorRight = grabberMotorRight;
        // we assume that one of these motors is set to reversed so that it goes opposite to the other one
        // this is why there are no negative signs in this code, even though one of the motors will be turning opposite

        this.ballHatchModePiston = ballHatchModePiston; // true -> ball mode
                                                        // false -> hatch mode
//        grabberMotorLeft.configContinuousCurrentLimit(40);
//        grabberMotorLeft.configPeakCurrentLimit(40);
//        grabberMotorLeft.configPeakCurrentDuration(200);
//        grabberMotorLeft.enableCurrentLimit(true);
    }


    public void expel() {
        if (this.isBallMode()) {
            this.expelBall();
        } else {
            this.expelHatch();
        }
    }

    public void expelBall() {
//        setBallMode(true);
        grabberMotorLeft.set(EXPEL_BALL_SPEED);
        grabberMotorRight.set(EXPEL_BALL_SPEED);
    }
    public void expelHatch() {
//        setBallMode(false);
        grabberMotorLeft.set(EXPEL_HATCH_SPEED);
        grabberMotorRight.set(EXPEL_HATCH_SPEED);
    }

    public void stop() {
        grabberMotorLeft.set(0);
        grabberMotorRight.set(0);
    }


    public void intake() {
        if (this.isBallMode()) {
            this.intakeBall();
        } else {
            this.intakeHatch();
        }
    }

    public void intakeBall() {
        grabberMotorLeft.set(BALL_INTAKE_SPEED);
        grabberMotorRight.set(BALL_INTAKE_SPEED);
    }
    public void intakeHatch() {
        grabberMotorLeft.set(HATCH_INTAKE_SPEED);
        grabberMotorRight.set(HATCH_INTAKE_SPEED);
    }

    public void holdBall() {
        grabberMotorLeft.set(BALL_INTAKE_SPEED / 2);
        grabberMotorRight.set(BALL_INTAKE_SPEED / 2);
    }

    public void holdHatch() {
        grabberMotorLeft.set(HATCH_INTAKE_SPEED / 2);
        grabberMotorRight.set(HATCH_INTAKE_SPEED / 2);
    }

    // convenience method to help readability
    public void setBallMode(boolean isBallMode) {
        setBallHatchModePiston(isBallMode);
    }

    public void setBallHatchModePiston(boolean on) {
        ballHatchModePiston.set(on);
    }
    public void toggleBallHatchMode() {
        ballHatchModePiston.set(!ballHatchModePiston.get());
    }

//    public void toggleHatchGrab() {
//        hatchGrabPiston.set(!hatchGrabPiston.get());
//    }
//
//    public void setHatchGrab(boolean on) {
//        hatchGrabPiston.set(on);
//    }
//    public void updatePistons() {
//        if (pancakeTimer.get() > 0.5) {
//            pancakeTimer.stop();
//            extendHatchMechPiston.set(false);
//        }
//    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ControlGrabber());
    }

    public boolean isBallMode() {
        return ballHatchModePiston.get();
    }

    public double getCurrentDraw() {
        return grabberMotorLeft.getCurrent();
    }
}
