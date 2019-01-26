/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TechnoTitan;
import frc.robot.sensors.NavXGyro;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class AutoAlign extends Command {
  private static final double WHEELBASE_WIDTH = 0.57; // width between wheels in meters
  private static final double MAX_VELOCITY = 3.9; // max velocity of path in m/s

  private final Trajectory.Config config;

  private DistanceFollower lFollower, rFollower;

  private NavXGyro gyro;


  // TODO: configure encoders and PID values
  private static final double kP = 1.0, kD = 0.0, A_GAIN = 0.0; // P, D, and A (acceleration gain) constants respectively

  private static final double kP_GYRO = 0.01;

  private static final double FINAL_DISTANCE = 0.5; // distance at which robot should be aligned and switch to fine adjustments, in meters

  public AutoAlign() {
    requires(TechnoTitan.drive);
    gyro = new NavXGyro();
    double speed = 0.3;
    config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, TechnoTitan.kDefaultPeriod, MAX_VELOCITY * speed, 2.0, 60.0);
  }

  public void generateTrajectoryFromVision() {
    if (!TechnoTitan.vision.canSeeTargets()) return;
    double xOffset = TechnoTitan.vision.getXOffset(),
           distance = TechnoTitan.vision.getYDistance(),
           skew = TechnoTitan.vision.getSkew();  // get angle, with 0 degrees = straight
    gyro.resetTo(skew);
    Waypoint[] points = new Waypoint[] {
      new Waypoint(1, -1, Math.toRadians(90)),
      new Waypoint(0, 2, Math.PI / 2)
    };

    Trajectory trajectory = Pathfinder.generate(points, config);
    TankModifier modifier = new TankModifier(trajectory).modify(WHEELBASE_WIDTH);

    lFollower = new DistanceFollower(modifier.getLeftTrajectory());
    rFollower = new DistanceFollower(modifier.getRightTrajectory());

    lFollower.configurePIDVA(kP, 0, kD, 1 / MAX_VELOCITY, A_GAIN);
    rFollower.configurePIDVA(kP, 0, kD, 1 / MAX_VELOCITY, A_GAIN);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    generateTrajectoryFromVision();
    TechnoTitan.drive.resetEncoders();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double inchesToMeters = 2.54 / 100;
    // Make sure no errors are thrown if it couldn't find the strips
    if (lFollower == null || rFollower == null) return;
    if (!lFollower.isFinished() || !rFollower.isFinished()) {
      SmartDashboard.putString("Align state", "Path");
      // If either one is finished, it just outputs 0 for "calculate", so it is safe to perform calculations even on finished followers
      double lOutput = lFollower.calculate(TechnoTitan.drive.getLeftEncoder().getDistance() * inchesToMeters);
      double rOutput = rFollower.calculate(TechnoTitan.drive.getRightEncoder().getDistance() * inchesToMeters);
      // headings are expressed as positive counterclockwise and 0 deg = right
      SmartDashboard.putString("Output", lOutput + ", " + rOutput);
      double heading = 90 - gyro.getAngle();
      double wantedHeading = Math.toDegrees(lFollower.getHeading() / 2 + rFollower.getHeading() / 2); // get the average desired heading
      
      SmartDashboard.putNumber("Wanted heading", wantedHeading);
      double error = Pathfinder.boundHalfDegrees(heading - wantedHeading); // positive if robot is too far counterclockwise
      TechnoTitan.drive.set(lOutput + kP_GYRO * error, rOutput - kP_GYRO * error);
    }
  }

  /**
   * This method is used so that if the robot spots something other than the
   * vision targets, it doesn't go beserk for a bit It should be called during the
   * last bit of the robot's journey (fine adjustments) as it makes assumptions
   * that it is in that part
   * 
   * @return true if the vision target parameters are reasonable with what we know
   *         about where the robot is
   */
  private boolean isVisionReasonable() {
    return TechnoTitan.vision.canSeeTargets()
      && TechnoTitan.vision.getYDistance() > FINAL_DISTANCE * 1.5 // We allow 50% error because this is just to filter bogus data
      && Math.abs(TechnoTitan.vision.getXOffset()) < 1 // Be less than 1 meter (approx 3 ft) from the center of the target
      && Math.abs(TechnoTitan.vision.getSkew()) < 30; // no more than 30 degrees angle error
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return lFollower == null || rFollower == null ||
            (lFollower.isFinished() && rFollower.isFinished());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    TechnoTitan.drive.stop();
  }
}
