package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.TalonSRX;

public class Elevator extends Subsystem {


    private static final double MAX_SPEED = 1.0;



    private DigitalInput lsTop;
    private DigitalInput lsBot;
    private TalonSRX m_motor;
    private boolean overrideLS = false;

    private Elevator() {}

    public Elevator(TalonSRX motor, DigitalInput limitSwitchTop, DigitalInput limitSwitchBottom) {
        this.lsTop = limitSwitchTop;
        this.lsBot = limitSwitchBottom;
        this.overrideLS = false;
        m_motor = motor;
    }

    public boolean areSensorsOverridden() {
        return this.overrideLS;
    }

    public void setOverrideSenors(boolean override) {
        this.overrideLS = override;
    }

    public void toggleOverrideSensors() {
        this.overrideLS = !this.overrideLS;
    }

    public void setSpeed(double speed) {
        // todo clamp/filter?
        // TODO LimitSwitch wrapper class
        // if we are not overriding the limit switches, and either is pressed
        if (!overrideLS && (lsTop.get() || lsBot.get())) {
            m_motor.set(0);
        } else {
            speed = Math.min(speed, MAX_SPEED);
            m_motor.set(speed);
        }
    }

    // todo reset encoders on every limitswitch hit

    public void resetEncoder() {
        this.m_motor.getEncoder().reset();
    }

    @Override
    protected void initDefaultCommand() {
        // controlelevator ?
    }
}
