package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motor.Filter;
import frc.robot.motor.Motor;
import frc.robot.sensors.gy521.Accel_GY521;

public class Arm extends Subsystem {

    // TODO: find value
    private static final double ROBOT_FRAME_WIDTH = 23.3;  // Distance in inches from arm pivot to distance sensor
    private Motor elbow, wrist;
    private Solenoid armSolenoid;

    private Accel_GY521 elbowSensor;
    private Accel_GY521 wristSensor;

    private Filter wristFilter, elbowFilter;



    private boolean isUp = false;
    // All inches
    private static final double ELBOW_LENGTH = 32.9,
                                WRIST_LENGTH = 16.5,
                                WRIST_TOP_HEIGHT = 13,
                                WRIST_BOTTOM_HEIGHT = 5;

    private static final double ELBOW_HEIGHT = 33.5;

    private static final double ELBOW_SENSOR_ANGLE_OFFSET = 17.16,
                                WRIST_ANGLE_ERROR = 2.5,
                                ELBOW_ANGLE_ERROR = 0;

    private static final double MAX_ELBOW_ANGLE = 70,
                                MIN_ELBOW_ANGLE = -70,
                                MAX_WRIST_ANGLE = 95,
                                MIN_WRIST_ANGLE = -110;

    private static final double RAMP_ELBOW = 20,
                                RAMP_WRIST = 20;

    // PID Loop tuning
    private static final double kWristP = 0.012;
    private static final double kWristI = 0.0004;
    private static final double kWristD = 0.0;

    private static final double kElbowP = 0.03;
    private static final double kElbowI = 0.001;
    private static final double kElbowD = 0.0;
    private static final double MAX_STEADY_VOLTAGE_ELBOW = 0.22;
    private static final double MAX_STEADY_VOLTAGE_WRIST = 0.08;

    public PIDAngleController wristController, elbowController;

    // TODO: change back
    private boolean overrideSensors = true;

    private double getDist(boolean isElbow) {
        double thetaB = Math.atan(WRIST_BOTTOM_HEIGHT / WRIST_LENGTH);
        double cElbow = -90, cWrist = Math.toDegrees(thetaB) - 90,
                rElbow = 90 + Math.asin((WRIST_LENGTH / Math.cos(thetaB) * 1 - ELBOW_HEIGHT) / ELBOW_LENGTH) * 180 / Math.PI,
                rWrist = 90 + Math.asin((-ELBOW_HEIGHT + ELBOW_LENGTH) / WRIST_LENGTH * Math.cos(thetaB)) * 180 / Math.PI;
        double d = getEllipseDist(cElbow, cWrist, rElbow, rWrist, isElbow);
        return  d;
    }

    private double getEllipseDist(double cElbow, double cWrist, double rElbow, double rWrist, boolean returnElbow) {
        double elbow = getElbowAngle(), wrist = getWristAngle();
        double diffElbow = elbow - cElbow, diffWrist = wrist - cWrist;
//        // (diffElbow * t)^2 / rElbow^2 + (diffWrist * t)^2 / rWrist^2 = 1
//        double t = 1 / Math.hypot(diffElbow / rElbow, diffWrist / rWrist);
//        double slopeElbow = 2 * diffElbow * t / (rElbow * rElbow),
//                slopeWrist = 2 * diffWrist * t / (rWrist * rWrist);
//        return returnElbow ? slopeWrist / slopeElbow * diffWrist * (1 - t) + diffElbow * (1 - t) : slopeElbow / slopeWrist * diffElbow * (1 - t) + diffWrist * (1 - t);
        // (x)^2 / rElbow^2 + (y)^2 / rWrist^2 = 1
        double x = returnElbow ? (1 - diffWrist * diffWrist / (rWrist * rWrist)) : (1 - diffElbow * diffElbow / (rElbow * rElbow));
        if (x < 0) return Double.POSITIVE_INFINITY;
        return (returnElbow ? diffElbow : diffWrist) - Math.sqrt(x) * (returnElbow ? rElbow : rWrist);
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
            SmartDashboard.putNumber("Elbow limited speed", speed);
        }
//        elbowFilter.update(speed);
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
            SmartDashboard.putNumber("Wrist limited speed", speed);
        }
//        wristFilter.update(speed);
        wrist.set(speed);
    }

    public Arm(Motor elbow, Motor wrist, Solenoid armPiston, Accel_GY521 elbowSensor, Accel_GY521 wristSensor) {
        this.elbow = elbow;
        this.wrist = wrist;
        this.armSolenoid = armPiston;

        this.elbowSensor = elbowSensor;
        this.wristSensor = wristSensor;

        wristFilter = new Filter(0.5);
        elbowFilter = new Filter(0.5);

        elbowController = new PIDAngleController("Elbow", kElbowP, kElbowI, kElbowD, MAX_STEADY_VOLTAGE_ELBOW, elbowAngleSensor, this::moveElbow, 0.035);
        wristController = new PIDAngleController("Wrist", kWristP, kWristI, kWristD, MAX_STEADY_VOLTAGE_WRIST, wristAngleSensor, this::moveWrist, 0.02);
        elbowController.setOutputRange(0.05, 0.6);
        wristController.setOutputRange(-0.5, 0.5);

        SmartDashboard.putData("Elbow", elbowController);
        SmartDashboard.putData("Wrist", wristController);
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
        return elbowSensor.getAngle() + ELBOW_SENSOR_ANGLE_OFFSET + ELBOW_ANGLE_ERROR;
    }

    public double getWristAngle() {
        return wristSensor.getAngle() + WRIST_ANGLE_ERROR;
    }


    @Override
    protected void initDefaultCommand() {
//        setDefaultCommand(new ControlArm());
    }

    public boolean areSensorsOverriden() {
        return overrideSensors;
    }

    public void toggleOverrideSensors() {
        overrideSensors = !overrideSensors;
    }

    public void updateElbowWristSetpoints() {
        this.wristController.updateSetpoint();
        this.elbowController.updateSetpoint();
    }

    public double getElbowOutput() {
        return elbow.getCurrent();
    }

    public double getWristOutput() {
        return wrist.getCurrent();
    }
}
