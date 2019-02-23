package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TechnoTitan;
import frc.robot.sensors.NavXGyro;
import frc.robot.sensors.TitanGyro;


public class Turn extends Command {
  private final double increasePerDegree = .005;
  private final double minSpeed = .5;
  private double turnSpeed;
  private double turnAngle;
  private Gyro gyro;

  public Turn(double turnAngle, double turnSpeed) {
    requires(TechnoTitan.drive);
    this.turnAngle = turnAngle;
    this.turnSpeed = turnSpeed;
//    gyro = new NavXGyro();
    gyro = new TitanGyro(TechnoTitan.centralGyro);
  }

  @Override
  protected void initialize() {
    gyro.reset();
  }

  private double getTargetAngle() {
    return turnAngle - Math.signum(turnAngle) * (30*turnSpeed);
  }

  @Override
  protected void execute() {
    double speed;
    double gyroAngle = Math.abs(gyro.getAngle());
    SmartDashboard.putNumber("Turn gyro (abs)", gyroAngle);
    SmartDashboard.putNumber("Target angle", getTargetAngle());
    if (turnSpeed > minSpeed) {
      if (gyroAngle < Math.abs(getTargetAngle()/2)) {
        speed = minSpeed + increasePerDegree * gyroAngle;
      } else {
        speed = minSpeed + increasePerDegree * (Math.abs(getTargetAngle()) - gyroAngle);
      }
      if (speed > turnSpeed) {
        speed = turnSpeed;
      }
    } else {
      speed = minSpeed; //if the min speed happens to be faster than the maxSpeed/turnSpeed provided value, fallback to minSpeed
    }
    // speed = turnSpeed;
    TechnoTitan.drive.turnInPlace(turnAngle > 0, speed);
    SmartDashboard.putNumber("Turn speed", speed);
  }

  @Override
  protected boolean isFinished() {
    if (timeSinceInitialized() < 0.2) return false;
    if (turnAngle > 0) {
      return gyro.getAngle() > getTargetAngle();
    } else {
      return gyro.getAngle() < getTargetAngle();
    }
  }

  @Override
  protected void end() {
    TechnoTitan.drive.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
