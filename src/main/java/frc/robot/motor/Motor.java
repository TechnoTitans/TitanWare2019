package frc.robot.motor;

import frc.robot.sensors.Encoder;;

public interface Motor {

	public void set(double speed);

	public double getPercentSpeed();
	
	public double getSpeed();

	public void stop();

	public void brake();

	public void coast();

	public boolean hasEncoder();

	public Encoder getEncoder();

	// public void setBrakeMode(boolean enable);

	public int getChannel();

	public boolean isReversed();
}
