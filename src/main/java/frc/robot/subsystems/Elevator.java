package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.TalonSRX;
import frc.robot.movements.elevator.ControlElevator;

public class Elevator extends Subsystem {


    private static final double MAX_ELEVATOR_SPEED = 1.0;
    private static final double MAX_WRIST_SPEED = 1.0;

    private DigitalInput lsTop;
    private DigitalInput lsBot;
    private TalonSRX m_motor;
    private boolean overrideLS = false;


    public static final class Constants {

    }

    private Elevator() {}

    public Elevator(TalonSRX motor, DigitalInput limitSwitchTop, DigitalInput limitSwitchBottom) {
        this.lsTop = limitSwitchTop;
        this.lsBot = limitSwitchBottom;
        this.overrideLS = false;

        // config pid
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

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

    public void moveElevator(double speed) {
        // todo clamp/filter?
        // TODO LimitSwitch wrapper class
        // if we are not overriding the limit switches, and either is pressed
        if (!overrideLS && (lsTop.get() || lsBot.get())) {
            m_motor.set(0);
        } else {
            speed = Math.min(speed, MAX_ELEVATOR_SPEED);
            m_motor.set(speed);
        }
    }

    public void moveWrist(double speed) {
        speed = Math.min(speed, MAX_WRIST_SPEED);
        m_motor.set(speedwr);
    }

    // todo reset encoders on every limitswitch hit

    public void resetEncoder() {
        this.m_motor.getEncoder().reset();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ControlElevator());
    }
}
