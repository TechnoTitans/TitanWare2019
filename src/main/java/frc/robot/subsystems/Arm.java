package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.motor.Motor;
import frc.robot.movements.arm.ControlArm;
import frc.robot.sensors.gy521.Accel_GY521;

public class Arm extends Subsystem {

    // TODO: find value
    private static final double ROBOT_FRAME_WIDTH = 23.3;  // Distance in inches from arm pivot to distance sensor
    private Motor elbow, wrist;
    private Solenoid armSolenoid;

    private Accel_GY521 elbowSensor;
    private Accel_GY521 wristSensor;



    private boolean isUp = false;

    // All inches
    private static final double ELBOW_LENGTH = 32.9,
                                WRIST_LENGTH = 16.5,
                                WRIST_TOP_HEIGHT = 13,
                                WRIST_BOTTOM_HEIGHT = 5;

    private static final double ELBOW_HEIGHT = 33.5;

    private static final double ELBOW_SENSOR_ANGLE_OFFSET = 17.16;

    private static final double MAX_ELBOW_ANGLE = 70,
                                MIN_ELBOW_ANGLE = -70,
                                MAX_WRIST_ANGLE = 95,
                                MIN_WRIST_ANGLE = -110;

    private static final double RAMP_ELBOW = 20,
                                RAMP_WRIST = 20;

    private boolean overrideSensors = false;

    private double getDist(boolean isElbow) {
        double thetaB = Math.atan(WRIST_BOTTOM_HEIGHT / WRIST_LENGTH);
        double cElbow = -90, cWrist = Math.toDegrees(thetaB) - 90,
                rElbow = 90 + Math.asin((WRIST_LENGTH / Math.cos(thetaB) * 1 - ELBOW_HEIGHT) / ELBOW_LENGTH) * 180 / Math.PI,
                rWrist = 90 + Math.asin((-ELBOW_HEIGHT + ELBOW_LENGTH) / WRIST_LENGTH * Math.cos(thetaB)) * 180 / Math.PI;
        double d = getEllipseDist(cElbow, cWrist, rElbow, rWrist, isElbow);
        return  (getWristAngle() <= cWrist && !isElbow) ? Double.POSITIVE_INFINITY : d;
    }

    private double getEllipseDist(double cElbow, double cWrist, double rElbow, double rWrist, boolean returnElbow) {
        double elbow = getElbowAngle(), wrist = getWristAngle();
        double diffElbow = elbow - cElbow, diffWrist = wrist - cWrist;
        // (diffElbow * t)^2 / rElbow^2 + (diffWrist * t)^2 / rWrist^2 = 1
        double t = 1 / Math.hypot(diffElbow / rElbow, diffWrist / rWrist);
//        double nearElbow = cElbow + diffElbow * t, nearWrist = cWrist + diffWrist * t;
        double slopeElbow = 2 * diffElbow * t / (rElbow * rElbow),
                slopeWrist = 2 * diffWrist * t / (rWrist * rWrist);
        return returnElbow ? slopeWrist / slopeElbow * diffWrist * (1 - t) + diffElbow * (1 - t) : slopeElbow / slopeWrist * diffElbow * (1 - t) + diffWrist * (1 - t);
    }

    public void moveElbow(double speed) {
        if (!overrideSensors) {
            double angle = getElbowAngle();
            if (angle > MAX_ELBOW_ANGLE - RAMP_ELBOW) {
                speed = Math.min(speed, (MAX_ELBOW_ANGLE - angle) / RAMP_ELBOW);
            }
            double minAngle = Math.max(angle - getDist(true), MIN_ELBOW_ANGLE);
            if (angle < minAngle + RAMP_ELBOW) {
                speed = Math.max(speed, -(angle - minAngle) / RAMP_ELBOW);
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
            double minAngle = Math.max(angle - getDist(false), MIN_WRIST_ANGLE);
            if (angle < minAngle + RAMP_WRIST) {
                speed = Math.max(speed, -(angle - minAngle) / RAMP_WRIST);
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
                - ROBOT_FRAME_WIDTH
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
            if (pidSource != PIDSourceType.kDisplacement)
                throw new UnsupportedOperationException();
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
            if (pidSource != PIDSourceType.kDisplacement)
                throw new UnsupportedOperationException();
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

    public boolean areSensorsOverriden() {
        return overrideSensors;
    }

    public void toggleOverrideSensors() {
        overrideSensors = !overrideSensors;
    }
}
