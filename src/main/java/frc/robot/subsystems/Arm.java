package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.Motor;
import frc.robot.movements.ControlArm;
import frc.robot.sensors.ArmAngleSensor;

public class Arm extends Subsystem {

    private Motor elbow, wrist;
    private Solenoid armSolenoid;

    public ArmAngleSensor elbowSensor;
    public ArmAngleSensor wristSensor;



    private boolean isUp = false;

    public static final double ELBOW_LENGTH = 32.7, // in
                               WRIST_LENGTH = 15; // in


    public void moveElbow(double speed) {
        // OI handles the deadband processing
        elbow.set(speed);
    }

    public void moveWrist(double speed) {
        wrist.set(speed);
    }

    public Arm(Motor elbow, Motor wrist, Solenoid armPiston, ArmAngleSensor elbowSensor, ArmAngleSensor wristSensor) {
        this.elbow = elbow;
        this.wrist = wrist;
        this.armSolenoid = armPiston;

        this.elbowSensor = elbowSensor;
        this.wristSensor = wristSensor;
    }


    public void setArmSolenoid(boolean on) {
        this.isUp = on;
        armSolenoid.set(on);
    }

    public void moveArmUp() {
        setArmSolenoid(true);
    }

    public void moveArmDown() {
        setArmSolenoid(false);
    }

    public void getCalculatedDistance() {
        // todo implement if needed
    }

    public void toggleUp() {
        this.setArmSolenoid(!isUp);
    }


    public boolean isUp() {
        return isUp;
    }


    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ControlArm());
    }
}
