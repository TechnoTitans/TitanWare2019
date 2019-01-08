/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.TechnoTitan;

/**
 * Add your docs here.
 */
public class NavXGyro implements Gyro {
    private double angle = 0;

    @Override
    public void close() throws Exception {
        TechnoTitan.navx.close();
    }

    @Override
    public void calibrate() {
        throw new UnsupportedOperationException("Cannot calibrate navx");
    }

    @Override
    public void reset() {
        angle = TechnoTitan.navx.getAngle();
    }

    @Override
    public double getAngle() {
        return TechnoTitan.navx.getAngle() - angle;
    }

    @Override
    public double getRate() {
        return TechnoTitan.navx.getRate();
    }

    @Override
    public void free() {
        TechnoTitan.navx.free();
    }

}
