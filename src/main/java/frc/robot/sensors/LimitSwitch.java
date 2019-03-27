package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {
    private DigitalInput limitSwitch;
    private boolean inverted;

    public LimitSwitch(DigitalInput limitSwitch, boolean inverted) {
        this.limitSwitch = limitSwitch;
        this.inverted = inverted;
    }

    public LimitSwitch(DigitalInput limitSwitch) {
        this(limitSwitch, false);
    }

    public boolean isPressed() {
        return limitSwitch.get() == !inverted;
    }
}
