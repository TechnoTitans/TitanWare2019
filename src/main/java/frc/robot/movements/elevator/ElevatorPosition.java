package frc.robot.movements.elevator;

public enum ElevatorPosition {
    DEFENSE(-90, 0),
    ROCKET_LEVEL_1_BALL(-80, 0),
    ROCKET_LEVEL_2_BALL(-70, 32),
    ROCKET_LEVEL_3_BALL(-70, 56),
    CARGO_SHIP_BALL(-70, 20.5),
    BALL_PICKUP(10, 0),
    LOW_HATCH(-90, 2.1),
    ROCKET_LEVEL_2_HATCH(-90, 32),
    ROCKET_LEVEL_3_HATCH(-90, 57.5),
    HATCH_PICKUP(-90, 0),
    STOW_POSITION(-70, 0);


    private double wristAngle, elevatorHeight;
    ElevatorPosition(double wristAngle, double elevatorHeight) {
        this.wristAngle = wristAngle;
        this.elevatorHeight = elevatorHeight;
    }

    public double getWristAngle() {
        return wristAngle;
    }

    public double getElevatorHeight() {
        return elevatorHeight;

    }
}
