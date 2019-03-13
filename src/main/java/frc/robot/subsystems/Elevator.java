package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.TalonSRX;
import frc.robot.movements.ConstantsMM;
import frc.robot.movements.elevator.ControlElevator;
import frc.robot.sensors.Encoder;
import frc.robot.sensors.QuadEncoder;

public class Elevator extends Subsystem {


    private static final double MAX_ELEVATOR_SPEED = 1.0,
                                MAX_WRIST_SPEED = 1.0,
                                MIN_ELEVATOR_SPEED = -0.5,
                                MIN_WRIST_SPEED = -0.7;
    private static final double MAX_TICKS = QuadEncoder.PULSES_PER_ROTATION * 2.5;

    // MARK - motion magic config
    private static final int TIMEOUT_MS = 30;

    private DigitalInput lsTop;
    private DigitalInput lsBot;
    private TalonSRX m_motor;
    private Encoder m_motorEncoder;

    //
    private double m_motorOffsetTicks = 0;


    // settings
    private boolean overrideLS = false;

    public Elevator(TalonSRX motor, DigitalInput limitSwitchTop, DigitalInput limitSwitchBottom) {
        this.lsTop = limitSwitchTop;
        this.lsBot = limitSwitchBottom;
        this.overrideLS = false;
        this.m_motorEncoder = motor.getEncoder();

        this.configMotionMagic(motor);

        m_motor = motor;
    }

    private void configMotionMagic(TalonSRX motor) {
        // config pid
        // todo extract constatns
        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, ConstantsMM.kElevatorPID, TIMEOUT_MS);
        // todo configselected filter
//        motor.setSensorPhase(true); // todo configure
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT_MS);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT_MS);

        motor.configNominalOutputForward(0, TIMEOUT_MS);
        motor.configNominalOutputReverse(0, TIMEOUT_MS);
        motor.configPeakOutputForward(1, TIMEOUT_MS);
//        motor.configPeakOutputForward(-1, TIMEOUT_MS);

        motor.configClosedLoopPeakOutput(0, MAX_ELEVATOR_SPEED);

        // MARK - PIDF stuff
        motor.selectProfileSlot(ConstantsMM.kElevatorSlot, ConstantsMM.kElevatorPID);
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
        if (!overrideLS && ((!lsTop.get() && speed < 0) || (!lsBot.get() && speed > 0))) {
            m_motor.set(0);
        } else {
            m_motor.set(speed);
        }
    }

    public void setMotorTarget(double target) {
        m_motor.set(ControlMode.MotionMagic, target + m_motorOffsetTicks);
    }

    // todo reset encoders on every limitswitch hit
    // todo actually use this
    public void compensateEncoder() {
        if (this.lsBot.get()) {
            m_motorOffsetTicks = m_motorEncoder.getRawPosition();
        } else if (this.lsTop.get()) {
            m_motorOffsetTicks = m_motorEncoder.getRawPosition() - MAX_TICKS;
        }
    }

    public double getPosition() {
        return m_motorEncoder.getRawPosition() - m_motorOffsetTicks;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ControlElevator());
    }
}
