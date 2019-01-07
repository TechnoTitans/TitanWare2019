// Do not make this a negative

package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;


public class Backwards extends Command {
  private double BackwardNum;
  public Backwards(double BackwardNum) {
    requires(TechnoTitan.drive);
    this.BackwardNum = BackwardNum;
  }

  @Override
  protected void initialize() {
    TechnoTitan.gyro.reset();
    TechnoTitan.drive.resetEncoders();
  }

  @Override
  protected void execute() {
    TechnoTitan.drive.set(-0.5, -0.5);
  }

  @Override
  protected boolean isFinished() {
    return TechnoTitan.drive.getLeftEncoder().getDistance() < -BackwardNum;
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
