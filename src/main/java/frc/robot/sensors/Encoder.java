package frc.robot.sensors;

public interface Encoder {

	double getDistance();
	
	double getSpeed();

	double getSpeedInches();

	void reset();

    double getRawPosition();
}
