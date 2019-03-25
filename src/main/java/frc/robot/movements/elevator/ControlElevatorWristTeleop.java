package frc.robot.movements.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ControlElevatorWristTeleop extends CommandGroup {
    public ControlElevatorWristTeleop() {
        addParallel(new ControlElevator());
        addParallel(new ControlWrist());
    }
}
