package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.TalonSRX;
import frc.robot.movements.elevator.ControlElevator;
import frc.robot.sensors.Encoder;
import frc.robot.sensors.LimitSwitch;

public class Elevator extends Subsystem {


    private static final double ELEVATOR_SPEED_UP = 18,  // in/s
                                ELEVATOR_SPEED_DOWN = 18;

    private static final double ELEVATOR_ACCEL_UP = 18,  // in/s^2
                                ELEVATOR_ACCEL_DOWN = 18;
    private static final double MAX_HEIGHT = 36;

    private static final double feedForward = 0.0;

    // MARK - motion magic config

    private LimitSwitch lsTop;
    private LimitSwitch lsBot;
    private TalonSRX m_motor;
    private Encoder m_motorEncoder;

    //
    private double m_motorOffsetTicks = 0;


    // settings
    private boolean overrideLS = false;

    public Elevator(TalonSRX motor, LimitSwitch limitSwitchTop, LimitSwitch limitSwitchBottom) {
        this.lsTop = limitSwitchTop;
        this.lsBot = limitSwitchBottom;
        this.overrideLS = false;
        this.m_motorEncoder = motor.getEncoder();

        motor.configPID(0.2, 0, 0, 0.2);

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
        // if we are not overriding the limit switches, and either is pressed
        if (!overrideLS && ((lsTop.isPressed() && speed > 0) || (lsBot.isPressed() && speed < 0))) {
            m_motor.set(0);
        } else {
            m_motor.set(speed);
        }
    }

    /**
     * Sets elevator target
     * @param target  target height in inches
     */
    public void setTargetHeight(double target) {
        boolean isMovingUp = target > m_motorEncoder.getDistance();
        double speed = isMovingUp ? ELEVATOR_SPEED_UP : ELEVATOR_SPEED_DOWN;
        double accel = isMovingUp ? ELEVATOR_ACCEL_UP : ELEVATOR_ACCEL_DOWN;
        m_motor.configMotionCruiseVelocity((int) (speed / (m_motorEncoder.getInchesPerPulse() * 10)));
        m_motor.configMotionAcceleration((int) (accel / (m_motorEncoder.getInchesPerPulse() * 10)));
        m_motor.set(ControlMode.MotionMagic, target / m_motorEncoder.getInchesPerPulse(), DemandType.ArbitraryFeedForward, feedForward);
    }

    // todo actually use this
    public void compensateEncoder() {
        if (this.lsBot.isPressed()) {
            m_motor.getEncoder().reset();
        } else if (this.lsTop.isPressed()) {
            m_motor.getEncoder().resetTo(MAX_HEIGHT);
        }
    }

    public double getPosition() {
        return m_motorEncoder.getRawPosition();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ControlElevator());
    }

    public double getHeight() {
        return m_motorEncoder.getDistance();
    }
}
