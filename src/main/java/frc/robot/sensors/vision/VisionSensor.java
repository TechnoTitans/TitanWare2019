/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TechnoTitan;
import frc.robot.sensors.TitanGyro;

/**
 * This class is a sort of wrapper class that simply communicates with the rpi, which is the real "Vision Sensor".
 */
public class VisionSensor {
    private static TitanGyro visionGyro;
    private static final double CAMERA_ROBOT_CENTER_OFFSET = 6;

    public VisionSensor() {
    }

    public static void initGyro() {
        visionGyro = new TitanGyro(TechnoTitan.centralGyro);
        visionGyro.resetTo(0);;
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
        return SmartDashboard.getNumber("pi-distance", 600) + CAMERA_ROBOT_CENTER_OFFSET;
    }

    /**
     * 
     * @return the angle of the robot relative to the strips in degrees, with clockwise being positive
     */
    public static double getNearestTargetAngle() {
        if (visionGyro == null) return 0;
//        return SmartDashboard.getNumber("pi-angle", 0.0);
//        return 0.0;
        final double ROCKET_ANGLE = 29;
        final double mid = (ROCKET_ANGLE + 90) / 2;
        double rawAngle = getRawAngle();
        double absAngle = Math.abs(rawAngle),
                isClockwise = rawAngle > 0 ? 1 : -1;
        if (absAngle < ROCKET_ANGLE / 2)
            return 0;
        else if (absAngle < mid)
            return ROCKET_ANGLE * isClockwise;
        else if (absAngle < 180 - mid)
            return 90 * isClockwise;
        else
            return (180 - ROCKET_ANGLE) * isClockwise;
    }

    public static double getRawAngle() {
        if (visionGyro == null) return 0;
        double rawAngle = ((visionGyro.getAngle() % 360) + 360) % 360;
        if (rawAngle > 180) rawAngle -= 360;
        return rawAngle;
    }

    public double getSkew() {
        return getRawAngle() - VisionSensor.getNearestTargetAngle();
    }

    /**
     * Call this method whenever we are guaranteed to be perfectly lined up with a target
     */
    public static void resetSkew() {
        visionGyro.resetTo(getNearestTargetAngle());
    }
    
    public boolean canSeeTargets() {
        return SmartDashboard.getBoolean("pi-detected", false);
    }

	public double getCenterOffset() {
		return SmartDashboard.getNumber("pi-c-offset", 0.0);
	}
}
