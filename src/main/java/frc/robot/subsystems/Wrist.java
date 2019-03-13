package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.TalonSRX;

public class Wrist extends Subsystem {
    private TalonSRX wristMotor;

    public Wrist(TalonSRX wristMotor) {
        this.wristMotor = wristMotor;
    }

    public void moveWrist(double speed) {
        this.wristMotor.set(speed);
    }

    public double getPosition() {
        return wristMotor.getEncoder().getRawPosition();
    }
    @Override
    protected void initDefaultCommand() {

    }
}
