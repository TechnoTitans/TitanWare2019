package frc.robot.movements.elevator;

public enum ElevatorPosition {
    ROCKET_LEVEL_1_BALL(-70, 0),
    ROCKET_LEVEL_2_BALL(-70, 31),
    ROCKET_LEVEL_3_BALL(-70, 59),
    CARGO_SHIP_BALL(-70, 21),
    BALL_PICKUP(2.6, 0),
    LOW_HATCH(-90, 0.6),
    ROCKET_LEVEL_2_HATCH(-90, 31),
    ROCKET_LEVEL_3_HATCH(-90, 60.8),
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
