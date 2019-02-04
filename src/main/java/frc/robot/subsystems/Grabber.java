package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.Motor;
import frc.robot.movements.ControlGrabber;

public class Grabber extends Subsystem {
    private Motor grabberMotor;
    private static final double EXPEL_SPEED = 0.4;
    private static final double INTAKE_SPEED = -0.4;

    public Grabber(Motor grabberMotor) {
        this.grabberMotor = grabberMotor;
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

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ControlGrabber());
    }
}
