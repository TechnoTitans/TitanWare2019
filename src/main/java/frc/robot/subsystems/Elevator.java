package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motor.TalonSRX;
import frc.robot.movements.elevator.ControlElevator;
import frc.robot.sensors.Encoder;
import frc.robot.sensors.LimitSwitch;

import java.util.ArrayList;
import java.util.List;

public class Elevator extends Subsystem {


    private static final double ELEVATOR_SPEED_UP = 12,  // in/s
                                ELEVATOR_SPEED_DOWN = 12;

    private static final double ELEVATOR_ACCEL_UP = 13,  // in/s^2
                                ELEVATOR_ACCEL_DOWN = 13;


    private static final double MAX_HEIGHT = 58.5;

    private static final double feedForward = 0.1;

    private LimitSwitch lsTop;
    private LimitSwitch lsBot;
    private TalonSRX m_motor;
    private Encoder m_motorEncoder;

    private static class FeedForwardDataCollector {
        private static class DataPoint {
            private double percentSpeed, measuredSpeed;
            DataPoint(double percentSpeed, double measuredSpeed) {
                this.percentSpeed = percentSpeed;
                this.measuredSpeed = measuredSpeed;
            }

            public String toString() {
                return percentSpeed + "," + measuredSpeed;
            }
        }
        private List<DataPoint> pts;
        private Timer timer;
        FeedForwardDataCollector() {
            pts = new ArrayList<>();
            timer = new Timer();
        }

        public void collect(double percentSpeed, double measuredSpeed) {
            if (timer.get() > 5) {
                pts.add(new DataPoint(percentSpeed, measuredSpeed));
                timer.reset();
                timer.start();
            }
        }

        public void putToDashboard(String key) {
            String[] result = new String[pts.size()];
            for (int i = 0; i < result.length; ++i) {
                result[i] = pts.get(i).toString();
            }
            SmartDashboard.putStringArray(key, result);
        }
    }

    private FeedForwardDataCollector data = new FeedForwardDataCollector();
    public void collectData() {
        data.collect(m_motor.getPercentSpeed(), m_motor.getSelectedSensorVelocity(0));
        data.putToDashboard("Elevator velocity data");
    }

    // settings
    private boolean overrideLS = false;

    public Elevator(TalonSRX motor, LimitSwitch limitSwitchTop, LimitSwitch limitSwitchBottom) {
        this.lsTop = limitSwitchTop;
        this.lsBot = limitSwitchBottom;
        this.overrideLS = false;
        this.m_motorEncoder = motor.getEncoder();
        m_motorEncoder.reset();

        motor.configPID(0.08, 0.001, 0.8, 2.5, (int) (4 / m_motorEncoder.getInchesPerPulse()));
        // 22680 / (75.375 - 13.25)
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
        double finalSpeed = speed;
        if (!overrideLS && ((lsTop.isPressed() && speed > 0) || (lsBot.isPressed() && speed < 0))) {
            finalSpeed = 0;
        }
//
//        if (overrideLS || !lsBot.isPressed()) {
//            finalSpeed += feedForward;
//        }
        m_motor.set(finalSpeed);
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
    }

    public double getHeight() {
        return m_motorEncoder.getDistance();
    }

    public boolean isAtBottom() {
        return lsBot.isPressed();
    }

    public boolean isAtTop() {
        return lsTop.isPressed();
    }

}
