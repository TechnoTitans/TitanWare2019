package frc.robot.movements.arm;


import frc.robot.subsystems.Arm;

public enum ArmPosition {
    // TODO: refine these
    ROCKET_LEVEL_1_BALL(-10.3, 0, false),
    ROCKET_LEVEL_2_BALL(42.2, 0, false),
    ROCKET_LEVEL_3_BALL(51, 28.8, true),
    ROCKET_LEVEL_2_HATCH(8.6, 90, false),
    ROCKET_LEVEL_3_HATCH(33.6,90,true),
    CARGO_SHIP_BALL(-28, 0, false),
    LOW_HATCH(-40, 90, false), // All hatch levels except higher hatches on rocket
    BALL_PICKUP(-35.8, -20.6, false);


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
