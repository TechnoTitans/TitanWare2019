package frc.robot.sensors;

public interface Encoder {

	double getDistance();
	
	double getSpeed();

	void reset();
}
