package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.Motor;
import frc.robot.movements.arm.ControlArm;
import frc.robot.sensors.gy521.Accel_GY521;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;

public class Arm extends Subsystem {

    private Motor elbow, wrist;
    private Solenoid armSolenoid;

    private Accel_GY521 elbowSensor;
    private Accel_GY521 wristSensor;



    private boolean isUp = false;

    // All inches
    private static final double ELBOW_LENGTH = 32.7,
                                WRIST_LENGTH = 16.5,
                                WRIST_TOP_HEIGHT = 13,
                                WRIST_BOTTOM_HEIGHT = 5;

    private static final double ELBOW_SENSOR_ANGLE_OFFSET = 21;

    private static final double MAX_ELBOW_ANGLE = 70,
                                MIN_ELBOW_ANGLE = -70,
                                MAX_WRIST_ANGLE = 110;

    private static final double RAMP_ELBOW = 10,
                                RAMP_WRIST = 10;

    private boolean overrideSensors = false;

    private double getMinElbow() {
        return Double.NEGATIVE_INFINITY;  // TODO: implement
    }

    private double getEllipseDist(double cElbow, double cWrist, double rElbow, double rWrist, boolean returnElbow) {
        double elbow = getElbowAngle(), wrist = getWristAngle();
        double diffElbow = elbow - cElbow, diffWrist = wrist - cWrist;
        // (diffElbow * t)^2 / rElbow^2 + (diffWrist * t)^2 / rWrist^2 = 1
        double t = 1 / Math.hypot(diffElbow / rElbow, diffWrist / rWrist);
        double nearElbow = cElbow + diffElbow * t, nearWrist = cWrist + diffWrist * t;
        return returnElbow ? (elbow - nearElbow) : (wrist - nearWrist);
    }

    public void moveElbow(double speed) {
        if (!overrideSensors) {
            double angle = getElbowAngle();
            if (angle > MAX_ELBOW_ANGLE - RAMP_ELBOW) {
                speed = Math.min(speed, (MAX_ELBOW_ANGLE - angle) / RAMP_ELBOW);
            }
            double minAngle = Math.max(getMinElbow(), MIN_ELBOW_ANGLE);
            if (angle < minAngle + RAMP_ELBOW) {
                speed = Math.max(speed, -(angle - MIN_ELBOW_ANGLE) / RAMP_ELBOW);
            }
        }
        elbow.set(speed);
    }

    public void moveWrist(double speed) {
        if (!overrideSensors) {
            double angle = getWristAngle();
            if (angle > MAX_WRIST_ANGLE + RAMP_WRIST) {
                speed = Math.min(speed, (MAX_WRIST_ANGLE - angle) / RAMP_WRIST);
            }

        }
        wrist.set(speed);
    }

    public Arm(Motor elbow, Motor wrist, Solenoid armPiston, Accel_GY521 elbowSensor, Accel_GY521 wristSensor) {
        this.elbow = elbow;
        this.wrist = wrist;
        this.armSolenoid = armPiston;

        this.elbowSensor = elbowSensor;
        this.wristSensor = wristSensor;
    }


    public void setArmSolenoid(boolean on) {
        this.isUp = on;
        armSolenoid.set(on);
    }

    /**
     * Returns the horizontal length of the arms
     * @param elbowAngle  Angle of elbow in degrees
     * @param wristAngle  Angle of wrist in degrees
     * @return  Horizontal distance
     */
    public static double getCalculatedDistance(double elbowAngle, double wristAngle) {
        return ELBOW_LENGTH * Math.cos(Math.toRadians(elbowAngle))
                + WRIST_LENGTH * Math.cos(Math.toRadians(wristAngle))
                + wristAngle < 0 ? WRIST_TOP_HEIGHT * Math.sin(Math.toRadians(wristAngle))
                    : WRIST_BOTTOM_HEIGHT * Math.sin(Math.toRadians(-wristAngle));
    }

    public void toggleUp() {
        this.setArmSolenoid(!isUp);
    }


    public boolean isUp() {
        return isUp;
    }

    public PIDSource elbowAngleSensor = new PIDSource() {
        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
            throw new NotImplementedException();
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }

        @Override
        public double pidGet() {
            return getElbowAngle();
        }
    };

    public PIDSource wristAngleSensor = new PIDSource() {
        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
            throw new NotImplementedException();
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }

        @Override
        public double pidGet() {
            return getWristAngle();
        }
    };

    public double getElbowAngle() {
        return elbowSensor.getAngle() + ELBOW_SENSOR_ANGLE_OFFSET;
    }

    public double getWristAngle() {
        return wristSensor.getAngle();
    }


    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ControlArm());
    }
}
