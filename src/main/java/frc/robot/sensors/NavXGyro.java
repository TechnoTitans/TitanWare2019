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

    /**
     * Resets the gyro so that the next reading of getAngle will be 0
     * Note: this is implemented so that each individual instance of NavxGyro can be reset without affecting other instances
     * This allows you to reset without worrying about what else is using the gyro
     */
    @Override
    public void reset() {
        angle = TechnoTitan.navx.getAngle();
    }

    /**
     * Resets such that the gyro reads angle
     * @param angle The angle that the next call of getAngle() should return
     */
    public void resetTo(double angle) {
        reset(); // sets the "0 angle" to the current direction
        this.angle -= angle; // subtract angle from the "0 angle"
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
        TechnoTitan.navx.close(); // free is depricated, and calls close anyways lmao
    }

}
