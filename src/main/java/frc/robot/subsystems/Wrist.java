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

    private static final double INIT_ANGLE = -93.4;

    private static final double degreesPerPulse = 360 / QuadEncoder.PULSES_PER_ROTATION;

    public Wrist(TalonSRX wristMotor) {
        this.wristMotor = wristMotor;
        wristMotor.configPID(4, 0.001, 0, 7, (int) (5 / degreesPerPulse));
        resetEncoder();
        wristMotor.configMotionCruiseVelocity((int) (WRIST_SPEED / (degreesPerPulse *  10)));
        wristMotor.configMotionAcceleration((int) (WRIST_ACCEL / (degreesPerPulse * 10)));

//        wristMotor.configContinuousCurrentLimit(40);
//        wristMotor.configPeakCurrentLimit(40);
//        wristMotor.configPeakCurrentDuration(200);
//        wristMotor.enableCurrentLimit(true);
    }

    public void resetEncoder() {
        wristMotor.getEncoder().resetToRaw((int) (INIT_ANGLE / degreesPerPulse));
    }

    public void moveWrist(double speed) {
        this.wristMotor.set(speed);
    }

    public double getPosition() {
        return wristMotor.getEncoder().getRawPosition();
    }

    public void setTargetAngle(double angle) {
        double target = angle / 360 * QuadEncoder.PULSES_PER_ROTATION;
        wristMotor.set(ControlMode.MotionMagic, target);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ControlWrist());
    }

    public double getAngle() {
        return wristMotor.getEncoder().getRawPosition() / QuadEncoder.PULSES_PER_ROTATION * 360;
    }
}
