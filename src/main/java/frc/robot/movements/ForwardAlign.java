package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TechnoTitan;
import frc.robot.sensors.NavXGyro;


public class ForwardAlign extends Command {
	private double slowdownDist;
	private double distEnd;
	
	private double speed;
	private double minSpeed = 0.1;

	private double kP_GYRO = 0.05;

	private Gyro gyro;
	private static final double kP_CAM = 0.05;


	public ForwardAlign(ArmPosition stage, double slowdownDist, double speed) {
		requires(TechnoTitan.drive);
		this.slowdownDist = slowdownDist;
		this.distEnd = stage.getHorizontalArmLength();
		distEnd = Math.max(distEnd, 11.9); // the sensor can't read closer than 11.9 in
		this.speed = speed;
		gyro = new NavXGyro();
	}

	protected void initialize() {
		gyro.reset();
	}

	protected void execute() {
		double distanceLeft = TechnoTitan.tfDistance.getDistance() - distEnd;
		double speed = minSpeed + (this.speed - minSpeed) * Math.min(1, distanceLeft / slowdownDist);
		double error;
		if (!isVisionReasonable()) error = gyro.getAngle() * kP_GYRO;
		else {
			error = TechnoTitan.vision.getCenterOffset() * kP_CAM;
			gyro.reset();
		}
		TechnoTitan.drive.set(speed - error, speed + error);
	}

	/**
	 * This method is used so that if the robot spots something other than the
	 * vision targets, it doesn't go beserk for a bit
	 * 
	 * @return true if the vision target parameters are reasonable with what we know
	 *         about where the robot is
	 */
	private boolean isVisionReasonable() {
		return TechnoTitan.vision.canSeeTargets()
		&& TechnoTitan.vision.getYDistance() / 100 * 2.54 < distEnd * 1.5 // We allow 50% error because this is just to filter bogus data
		&& Math.abs(TechnoTitan.vision.getXOffset()) < 1 // Be less than 1 meter (approx 3 ft) from the center of the target
		&& Math.abs(TechnoTitan.vision.getSkew()) < 30; // no more than 30 degrees angle error
	}


	@Override
	protected boolean isFinished() {
		return TechnoTitan.tfDistance.getDistance() <= distEnd;
	}

	@Override
	protected void end() {
		TechnoTitan.drive.stop();
		SmartDashboard.putNumber("End value", TechnoTitan.tfDistance.getDistance());
	}
}
