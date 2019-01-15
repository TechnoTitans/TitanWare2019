package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.Motor;
import frc.robot.movements.ControlArm;

public class Arm extends Subsystem {

    private Motor elbow, wrist;
    private Solenoid armSolenoid;

    public static final double ARM_LENGTH = 32.7, // in
                                WRIST_LENGTH = 10; // in

    public Arm(Motor elbow, Motor wrist, Solenoid armPiston) {
        this.elbow = elbow;
        this.wrist = wrist;
        this.armSolenoid = armPiston;
    }

    public void moveElbow(double speed) {
      if(speed > 0.01 || speed < -0.01){
        elbow.set(speed);
      } else {
        elbow.set(0);
      }
    }

    public void moveWrist(double speed) {
      if(speed > 0.01 || speed < -0.01){
        wrist.set(speed);
      } else {
        wrist.set(0);
      }
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
