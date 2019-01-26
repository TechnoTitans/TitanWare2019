package frc.robot.movements;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.TechnoTitan;
import frc.robot.sensors.NavXGyro;


public class ForwardAlign extends Command {
	private double slowdownDist;
	private double distEnd;
	
	private double speed;
	private double minSpeed = 0.1;

	private double kP_GYRO = 0.05;

	private Gyro gyro;


	public ForwardAlign(ArmPosition stage, double slowdownDist, double speed) {
		requires(TechnoTitan.drive);
		this.slowdownDist = slowdownDist;
		this.distEnd = stage.getHorizontalArmLength();
		distEnd = Math.max(distEnd, 11.9); // the sensor can't read closer than 11.9 in
		this.speed = speed;
	}

	protected void initialize() {
		gyro = new NavXGyro();
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
