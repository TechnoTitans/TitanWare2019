package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.TalonSRX;
import frc.robot.movements.ConstantsMM;
import frc.robot.movements.elevator.ControlElevator;
import frc.robot.sensors.QuadEncoder;

public class Elevator extends Subsystem {


    private static final double MAX_ELEVATOR_SPEED = 1.0;
    private static final double MAX_WRIST_SPEED = 1.0;
    private static final double MAX_TICKS = QuadEncoder.PULSES_PER_ROTATION * 2.5;

    // MARK - motion magic config
    private static final int TIMEOUT_MS = 30;

    private DigitalInput lsTop;
    private DigitalInput lsBot;
    private TalonSRX m_motor;
    private QuadEncoder m_motorEncoder;

    //
    private double m_motorOffsetTicks = 0;


    // settings
    private boolean overrideLS = false;


    public static final class Constants {

    }

    public Elevator(TalonSRX motor, DigitalInput limitSwitchTop, DigitalInput limitSwitchBottom) {
        this.lsTop = limitSwitchTop;
        this.lsBot = limitSwitchBottom;
        this.overrideLS = false;
        this.m_motorEncoder = (QuadEncoder) motor.getEncoder();

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
        motor.configPeakOutputForward(-1, TIMEOUT_MS);

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
        if (!overrideLS && (lsTop.get() || lsBot.get())) {
            m_motor.set(0);
        } else {
            speed = Math.min(speed, MAX_ELEVATOR_SPEED);
            m_motor.set(speed);
        }
    }

    public void moveWrist(double speed) {
        speed = Math.min(speed, MAX_WRIST_SPEED);
        m_motor.set(speed);
    }



    // todo reset encoders on every limitswitch hit
    // todo actually use this
    public void compensateEncoder() {
        if (this.lsBot.get()) {
            m_motorOffsetTicks = -1 * m_motorEncoder.getRawPosition();
        } else if (this.lsTop.get()) {
            m_motorOffsetTicks = MAX_TICKS - m_motorEncoder.getRawPosition();
        }
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ControlElevator());
    }
}
