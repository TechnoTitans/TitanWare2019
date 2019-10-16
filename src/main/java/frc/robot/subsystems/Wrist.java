package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.TalonSRX;
import frc.robot.movements.elevator.ControlWrist;
import frc.robot.sensors.QuadEncoder;

public class Wrist extends Subsystem {
    private TalonSRX wristMotor;

    private static final double WRIST_SPEED = 90;  // degrees per second
    private static final double WRIST_ACCEL = 35;  // degrees/second/second

    private static final double INIT_ANGLE = -90;

    // private static final double degreesPerPulse = 360 / QuadEncoder.PULSES_PER_ROTATION;
    private static final double degreesPerPulse = 0.11703511;

    public Wrist(TalonSRX wristMotor) {
        this.wristMotor = wristMotor;
//        wristMotor.configPID(2, 0.001, 20, 7, (int) (5 / degreesPerPulse));
        wristMotor.configPID(3, 0.001, 10, 7, (int) (5 / degreesPerPulse));

        resetEncoder();
        wristMotor.configMotionCruiseVelocity((int) (WRIST_SPEED / (degreesPerPulse *  10)));
        wristMotor.configMotionAcceleration((int) (WRIST_ACCEL / (degreesPerPulse * 10)));
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
        double target = angle / degreesPerPulse;
        wristMotor.set(ControlMode.MotionMagic, target);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public double getAngle() {
        return wristMotor.getEncoder().getRawPosition() * degreesPerPulse;
    }
}
