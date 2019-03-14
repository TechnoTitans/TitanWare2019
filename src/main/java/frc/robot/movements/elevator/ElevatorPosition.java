package frc.robot.movements.elevator;

public enum ElevatorPosition {
    ROCKET_LEVEL_1_BALL(90, 10),
    ROCKET_LEVEL_2_BALL(90, 20),
    CARGO_SHIP_BALL(90, 15),
    BALL_PICKUP(-20, 0),
    LOW_HATCH(90, 5),
    ROCKET_LEVEL_2_HATCH(0, 10),
    HATCH_PICKUP(90, 0),
    STOW_POSITION(90, 0);

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
