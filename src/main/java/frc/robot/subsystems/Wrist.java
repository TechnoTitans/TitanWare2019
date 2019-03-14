package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.TalonSRX;
import frc.robot.movements.elevator.ControlWrist;
import frc.robot.sensors.QuadEncoder;

public class Wrist extends Subsystem {
    private TalonSRX wristMotor;

    private static final double WRIST_SPEED = 50;  // degrees per second
    private static final double WRIST_ACCEL = 50;  // degrees/second/second

    public Wrist(TalonSRX wristMotor) {
        this.wristMotor = wristMotor;
        wristMotor.configPID(0.2, 0, 0, 0.3);
        wristMotor.getEncoder().reset();
        wristMotor.configMotionCruiseVelocity((int) (WRIST_SPEED * QuadEncoder.PULSES_PER_ROTATION / (360 * 10)));
        wristMotor.configMotionAcceleration((int) (WRIST_ACCEL * QuadEncoder.PULSES_PER_ROTATION) / (360 * 10));
    }

    public void moveWrist(double speed) {
        this.wristMotor.set(speed);
    }

    public double getPosition() {
        return wristMotor.getEncoder().getRawPosition();
    }

    public void setTargetAngle(double angle) {
        double target = (angle - 90) / 360 * QuadEncoder.PULSES_PER_ROTATION;
        wristMotor.set(ControlMode.MotionMagic, target);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ControlWrist());
    }

    public double getAngle() {
        return wristMotor.getEncoder().getRawPosition() / QuadEncoder.PULSES_PER_ROTATION * 360 + 90;
    }
}
