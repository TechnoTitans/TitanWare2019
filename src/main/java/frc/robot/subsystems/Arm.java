package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.Motor;
import frc.robot.movements.ControlArm;

public class Arm extends Subsystem {

//    private final AnalogAccelerometer accelerometer;
    private Motor elbow, wrist;
    private Solenoid armSolenoid;

    public static final double ARM_LENGTH   = 32.7, // in
                               WRIST_LENGTH = 10; // in

    public void moveElbow(double speed) {
        // OI handles the deadband processing
        elbow.set(speed);
    }

    public void moveWrist(double speed) {
        wrist.set(speed);
    }

    public Arm(Motor elbow, Motor wrist, Solenoid armPiston) {
        this.elbow = elbow;
        this.wrist = wrist;
        this.armSolenoid = armPiston;
    }


    public void setArmSolenoid(boolean on) {
        armSolenoid.set(on);
    }

    public void moveArmUp() {
        setArmSolenoid(true);
    }

    public void moveArmDown() {
        setArmSolenoid(false);
    }



    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ControlArm());
    }
}
