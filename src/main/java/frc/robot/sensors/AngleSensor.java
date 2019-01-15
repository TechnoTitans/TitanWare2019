package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;

public class AngleSensor extends AnalogInput implements PIDSource {

    private double sensorValue;

    /**
     * Construct an analog channel.
     *
     * @param channel The channel number to represent. 0-3 are on-board 4-7 are on the MXP port.
     */
    public AngleSensor(int channel) {
        super(channel);
    }


    public double getAngle() {
        return 0; // todo calculate
    }
}
