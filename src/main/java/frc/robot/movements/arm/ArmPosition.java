package frc.robot.movements.arm;


import frc.robot.subsystems.Arm;

public enum ArmPosition {
    ROCKET_LEVEL_1(90, 90, false, 36),
    ROCKET_LEVEL_2(90, 90, false),
    ROCKET_LEVEL_3(90, 90, true); // TODO figure out optimal configuration for these

    private double elbowAngle, wristAngle; // DEGREES
    private boolean solenoidEnabled;
    private double horizontalArmLength;

    ArmPosition(double elbowAngle, double wristAngle, boolean solenoidEnabled) {
        this(elbowAngle, wristAngle, solenoidEnabled,  Arm.getCalculatedDistance(elbowAngle, wristAngle));
    }

    ArmPosition(double elbowAngle, double wristAngle, boolean solenoidEnabled, double horizontalArmLength) {
        this.elbowAngle = elbowAngle;
        this.wristAngle = wristAngle;
        this.solenoidEnabled = solenoidEnabled;
        this.horizontalArmLength = horizontalArmLength;
        
    }

    public double getHorizontalArmLength() {
        return horizontalArmLength;
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
