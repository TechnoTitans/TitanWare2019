package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {
    private final Solenoid climbSolenoid;

    public Climber(Solenoid climbSolenoid) {
        this.climbSolenoid = climbSolenoid;
    }

    public void toggle() {
        climbSolenoid.set(!climbSolenoid.get());
    }
    @Override
    protected void initDefaultCommand() {

    }

    public boolean isClimbing() {
        return climbSolenoid.get();
    }
}
