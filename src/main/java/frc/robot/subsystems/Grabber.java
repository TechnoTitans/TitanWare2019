package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.Motor;
import frc.robot.movements.ControlGrabber;

public class Grabber extends Subsystem {
    private Motor grabberMotor;
    private Timer pancakeTimer;
    private static final double EXPEL_SPEED = 1;
    private static final double INTAKE_SPEED = -0.4;

    private Solenoid pancakePistons;
    private Solenoid grabbyPiston;

    public Grabber(Motor grabberMotor, Solenoid pancakePistons, Solenoid grabbyPiston) {
        this.grabberMotor = grabberMotor;
        this.pancakePistons = pancakePistons;
        this.grabbyPiston = grabbyPiston;
        pancakeTimer = new Timer();
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

    public void setSpeed(double speed) {
        grabberMotor.set(speed);
    }

    public void expelHatch() {
        pancakePistons.set(true);
        pancakeTimer.reset();
        pancakeTimer.start();
    }

    public void setPancakePistons(boolean on) {
        pancakePistons.set(on);
    }

    public void setClawPistons(boolean on) {
        grabbyPiston.set(on);
    }

    public void toggleClawPistons() {
        grabbyPiston.set(!grabbyPiston.get());
    }

    public void updatePistons() {
        if (pancakeTimer.get() > 0.5) {
            pancakeTimer.stop();
            pancakePistons.set(false);
        }
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ControlGrabber());
    }
}
