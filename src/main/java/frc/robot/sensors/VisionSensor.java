/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

/**
 * Add your docs here.
 */
public class VisionSensor {
    public VisionSensor() {

    }

    /**
     * 
     * @return the x offset to the vision targets in meters (not inches!), when the robot is left of the strips, it is negative
     */
    public double getXOffset() {
        return 0.0;
    }

    /**
     * @return the y distance from the tarrgets in meters (not inches!)
     */
    public double getYDistance() {
        return 0.0;
    }

    /**
     * 
     * @return the angle of the robot relative to the strips in degrees, with clockwise being positive
     */
    public double getSkew() {
        return 0.0;
    }
    
    public boolean canSeeTargets() {
        return false;
    }
}
