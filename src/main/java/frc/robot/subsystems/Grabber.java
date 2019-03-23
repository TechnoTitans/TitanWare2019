package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.Motor;
import frc.robot.motor.TalonSRX;
import frc.robot.movements.ControlGrabber;

public class Grabber extends Subsystem {

    private Motor grabberMotor;

//    private Timer pancakeTimer;
    private static final double EXPEL_SPEED = 1;
    private static final double INTAKE_SPEED = -1;

    private Solenoid extendHatchMechPiston, hatchGrabPiston;

    public Grabber(TalonSRX grabberMotor, Solenoid extendHatchMechPiston, Solenoid hatchGrabPiston) {
        this.grabberMotor = grabberMotor;

        this.extendHatchMechPiston = extendHatchMechPiston;
        this.hatchGrabPiston = hatchGrabPiston;
//        pancakeTimer = new Timer();
//        grabberMotor.configContinuousCurrentLimit(40);
//        grabberMotor.configPeakCurrentLimit(40);
//        grabberMotor.configPeakCurrentDuration(200);
//        grabberMotor.enableCurrentLimit(true);
    }

    public void expel() {
        grabberMotor.set(EXPEL_SPEED);
    }

    public void stop() {
        grabberMotor.set(0);
    }

    public void intake() {
        grabberMotor.set(INTAKE_SPEED);
    }

    public void setGrabberMotor(double speed) {
        grabberMotor.set(speed);
    }


    public void setExtendHatchMechPiston(boolean on) {
        extendHatchMechPiston.set(on);
    }
    public void toggleExtendHatchMechPiston() {
        extendHatchMechPiston.set(!extendHatchMechPiston.get());
    }

//    public void setClawPistons(boolean on) {
//        grabbyPiston.set(on);
//    }

//    public void toggleClawPistons() {
//        grabbyPiston.set(!grabbyPiston.get());
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


}
