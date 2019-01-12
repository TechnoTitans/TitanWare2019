package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.motor.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Elbow extends Subsystem {
  private TalonSRX elbowTalon;

  public Elbow(TalonSRX elbowTalon) {
    this.elbowTalon = elbowTalon;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    //IDK mate
  }

  public void moveUp(double speed){
    if(speed > 0.01 || speed < -0.01){
      elbowTalon.set(speed);
    } else {
      elbowTalon.set(0);
    }
  }
}
