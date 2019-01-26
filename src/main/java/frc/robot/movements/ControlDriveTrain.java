/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.TechnoTitan;
import frc.robot.motor.Filter;

public class ControlDriveTrain extends Command {
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
    leftFilter.update(TechnoTitan.oi.getLeft());
    rightFilter.update(TechnoTitan.oi.getRight());
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
