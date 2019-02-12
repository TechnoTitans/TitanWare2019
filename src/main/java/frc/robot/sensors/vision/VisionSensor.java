/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is a sort of wrapper class that simply communicates with the rpi, which is the real "Vision Sensor".
 */
public class VisionSensor {
    public VisionSensor() {

    }

    public void startRecording() {
        SmartDashboard.putBoolean("pi-record", true);
    }

    public void stopRecording() {
        SmartDashboard.putBoolean("pi-record", false);
    }

    /**
     * 
     * @return the x offset to the vision targets in inches, when the robot is left of the strips, it is negative
     */
    public double getXOffset() {
        return SmartDashboard.getNumber("pi-x-offset", 0.0);
    }

    /**
     * @return the y distance from the tarrgets in inches
     */
    public double getYDistance() {
        return SmartDashboard.getNumber("pi-distance", 600);
    }

    /**
     * 
     * @return the angle of the robot relative to the strips in degrees, with clockwise being positive
     */
    public double getSkew() {
        return SmartDashboard.getNumber("pi-angle", 0.0);
    }
    
    public boolean canSeeTargets() {
        return SmartDashboard.getBoolean("pi-detected", false);
    }

	public double getCenterOffset() {
		return SmartDashboard.getNumber("pi-c-offset", 0.0);
	}
}