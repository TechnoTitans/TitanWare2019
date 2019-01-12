package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.Motor;
import frc.robot.motor.TalonSRX;

public class Arm extends Subsystem {

    private Motor elbow, wrist;

    public Arm(Motor elbow, Motor wrist) {
        this.elbow = elbow;
        this.wrist = wrist;
    }

    public void moveElbow(double speed) {

    }

    public void moveWrist(double speed) {

    }


    @Override
    protected void initDefaultCommand() {

    }
}
