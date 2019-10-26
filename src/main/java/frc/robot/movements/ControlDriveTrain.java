/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;

public class ControlDriveTrain extends Command {

  private static final double MAX_TURN_SPEED = 0.5;

  private Filter leftFilter, rightFilter;

  public ControlDriveTrain() {
    requires(TechnoTitan.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    leftFilter = new Filter(0.1);
    rightFilter = new Filter(0.1);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double maxSpeed = TechnoTitan.oi.getSlowdown() ? 0.25 : 1;
    double joystickLeft = TechnoTitan.oi.getLeft();
    double joystickRight = TechnoTitan.oi.getRight();

    if (TechnoTitan.climber.isClimbing()) maxSpeed *= 0.3;

    // MARK - Limit speed while turning
    if (Math.signum(joystickLeft) == -Math.signum(joystickRight)) {
      // (above) if the joystick values are opposite signs, then the operator is turning/rotating the robot

      // here, we set adjust the speeds
      // the first part of the code preserves the sign of the input, which means that it preserves the direction the joystick is facing (up or down)
      // the second part looks at simply the magnitude of the joystick input, and selects which ever is smaller, the input or the maximum speed
      // thus restricting the speed to the desired maximum value
      joystickLeft = Math.signum(joystickLeft) * Math.min(Math.abs(joystickLeft), MAX_TURN_SPEED);
      joystickRight = Math.signum(joystickRight) * Math.min(Math.abs(joystickRight), MAX_TURN_SPEED);
    }

    leftFilter.update(joystickLeft * maxSpeed);
    rightFilter.update(joystickRight * maxSpeed);
    TechnoTitan.drive.set(leftFilter.getValue(), rightFilter.getValue());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    TechnoTitan.drive.stop();
  }
}
