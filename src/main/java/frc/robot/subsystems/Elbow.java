package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.motor.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Elbow extends Subsystem {
  private TalonSRX elbowTalon;

  public Elbow(TalonSRX)
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
