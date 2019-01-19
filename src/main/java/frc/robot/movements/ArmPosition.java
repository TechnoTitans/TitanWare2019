package frc.robot.movements;


import frc.robot.subsystems.Arm;

public enum ArmPosition {
    ROCKET_LEVEL_1(90, 90, false),
    ROCKET_LEVEL_2(90, 90, false),
    ROCKET_LEVEL_3(90,90,true); // TODO figure out optimal configuration for these

    private double elbowAngle, wristAngle; // DEGREES
    private boolean solenoidEnabled;

    ArmPosition(double elbowAngle, double wristAngle, boolean solenoidEnabled) {
        this.elbowAngle = elbowAngle;
        this.wristAngle = wristAngle;
        this.solenoidEnabled = solenoidEnabled;
    }

    // in reference to where it attaches to on elevator to end of wrist
    public double getHorizontalArmLength() {
        return Arm.ELBOW_LENGTH * Math.sin(Math.toRadians(elbowAngle)) + Arm.WRIST_LENGTH * Math.cos(Math.toRadians(wristAngle));
    }

    public double getElbowAngle() {
        return elbowAngle;
    }

    public double getWristAngle() {
        return wristAngle;
    }

    public boolean isSolenoidEnabled() {
        return solenoidEnabled;
    }
}
