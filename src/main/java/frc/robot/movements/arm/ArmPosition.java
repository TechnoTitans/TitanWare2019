package frc.robot.movements.arm;


import frc.robot.subsystems.Arm;

public enum ArmPosition {
    // TODO: refine these
    ROCKET_LEVEL_1_BALL(-24, 15, false),
//    ROCKET_LEVEL_1_BALL(-41, 20, false),
    ROCKET_LEVEL_2_BALL(30, 18, false),
//    ROCKET_LEVEL_2_BALL(13, 20, false),
    ROCKET_LEVEL_3_BALL(46, 40, true),
    ROCKET_LEVEL_2_HATCH(8.6, 90, false),
    ROCKET_LEVEL_3_HATCH(33.6,90,true),
    CARGO_SHIP_BALL(10, 0, false),
    CARGO_SHIP_BALL_2(-18, 51, false),
//    LOW_HATCH(-40, 85, false, 26), // All hatch levels except higher hatches on rocket
    LOW_HATCH(-16, 0, false),
    BALL_PICKUP(-39, -20, false),
    HATCH_PICKUP(-56.0, 3, false),
    STOW_POSITION(-56, 88, false);


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
