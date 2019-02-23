package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;
import frc.robot.movements.arm.ArmPosition;
import frc.robot.sensors.NavXGyro;
import frc.robot.sensors.TitanGyro;


public class ForwardAlign extends Command {
	private static final double MIN_READABLE_DISTANCE = 11.9;
	private double slowdownDist;
	private double distEnd;
	
	private double speed;
	private double minSpeed = 0.1;

	private double kP_GYRO = 0.05;

	private TitanGyro gyro;
	private static final double kP_CAM = 0.05;


	public ForwardAlign(ArmPosition stage, double slowdownDist, double speed) {
		requires(TechnoTitan.drive);
		this.slowdownDist = slowdownDist;
		this.distEnd = stage.getHorizontalArmLength();
		distEnd = Math.max(distEnd, MIN_READABLE_DISTANCE); // the sensor can't read closer than 11.9 in
		this.speed = speed;
//		gyro = new NavXGyro();
		gyro = new TitanGyro(TechnoTitan.centralGyro);
	}

	protected void initialize() {
		gyro.reset();
	}

	protected void execute() {
		double distanceLeft = TechnoTitan.tfDistance.getDistance() - distEnd;
		double speed = minSpeed + (this.speed - minSpeed) * Math.min(1, distanceLeft / slowdownDist);
		double error = gyro.getAngle() * kP_GYRO;
		TechnoTitan.drive.set(speed - error, speed + error);
	}


	@Override
	protected boolean isFinished() {
		return TechnoTitan.tfDistance.getDistance() <= distEnd;
	}

	@Override
	protected void end() {
		TechnoTitan.drive.stop();
	}
}
