/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * Add your docs here.
 */
public class NavXGyro implements Gyro {
    private AHRS m_internalSensor;

    public NavXGyro(AHRS internalGyro) {
        this.m_internalSensor = internalGyro;
    }

    @Override
    public void close() throws Exception {
        this.m_internalSensor.close();
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
        this.m_internalSensor.reset();
    }

//    /**
//     * Resets such that the gyro reads angle
//     * @param angle The angle that the next call of getAngle() should return
//     */
//
//    public void resetTo(double angle) {
//        reset(); // sets the "0 angle" to the current direction
//        this.angle -= angle; // subtract angle from the "0 angle"
//    }

    @Override
    public double getAngle() {
        return m_internalSensor.getAngle();
    }

    @Override
    public double getRate() {
        return this.m_internalSensor.getRate();
    }

    @Override
    public void free() {
        this.m_internalSensor.close();
    }

}
