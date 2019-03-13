package frc.robot.sensors;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.motor.TalonSRX;

/**
 * Encoder class. Used to measure how far the robot traveled
 */

public class QuadEncoder implements Encoder {

	private TalonSRX talonSRX;
	public static final double PULSES_PER_ROTATION = 4096;
	private double inchesPerPulse; // configure
	
	public QuadEncoder(TalonSRX talonSRX, double inchesPerPulse, boolean reversed) {
		this.talonSRX = talonSRX;
		this.inchesPerPulse = inchesPerPulse;
		// this.talonSRX.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		this.talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		talonSRX.setSensorPhase(reversed);
	}

	/**
	 * The total distance that the motor has traveled
	 * 
	 * @return total distance
	 */
	@Override
	public double getDistance() {
		return talonSRX.getSelectedSensorPosition(0) * inchesPerPulse;
	}

	/**
	 * Gets speed of the TalonSRX in RPM
	 */
	// speed = enc counts / 100 ms
	// (speed * 60 secs)
	// --------------------------------------
	// 4096 encoder counts * 100 milliseconds
	@Override
	public double getSpeed() {
		return (talonSRX.getSelectedSensorVelocity(0) * 60) / (PULSES_PER_ROTATION * 0.1);
	}

	@Override
	public double getSpeedInches() {
		return talonSRX.getSelectedSensorVelocity(0) * 10 * inchesPerPulse;
	}

	@Override
	public void reset() {
		talonSRX.setSelectedSensorPosition(0, 0, 0);
	}

	public double getRawPosition() {
		return talonSRX.getSelectedSensorPosition(0);
	}

	public TalonSRX getTalon() {
		return talonSRX;
	}
}
