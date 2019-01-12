package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.motor.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;


public class Wrist extends Subsystem {
  private TalonSRX wristTalon;


public Wrist(TalonSRX wristTalon) {
  this.wristTalon = wristTalon;
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void moveUp(double speed){
    if(speed > 0.01 || speed < -0.01){
      wristTalon.set(speed);
    } else {
      wristTalon.set(0);
    }
  }
}
