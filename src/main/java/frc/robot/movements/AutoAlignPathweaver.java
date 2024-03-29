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
import frc.robot.sensors.TitanGyro;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class AutoAlignPathweaver extends Command {
  private static final double WHEELBASE_WIDTH = 0.57; // width between wheels in meters
  private static final double MAX_VELOCITY = 3.9; // max velocity of path in m/s

  private final Trajectory.Config config;

  private DistanceFollower lFollower, rFollower;

  private TitanGyro gyro;


  // TODO: configure encoders and PID values
  private static final double kP = 1.0, kD = 0.0, A_GAIN = 0.0; // P, D, and A (acceleration gain) constants respectively

  private static final double kP_GYRO = 0.01;

  private static final double FINAL_DISTANCE = 0.5; // distance at which robot should be aligned and switch to fine adjustments, in meters

  public AutoAlignPathweaver() {
    requires(TechnoTitan.drive);
//    gyro = new NavXGyro();
    gyro = new TitanGyro(TechnoTitan.centralGyro);
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
