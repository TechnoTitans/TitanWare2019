package frc.robot.movements.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.TechnoTitan;

public class MoveArmPiston extends Command {
    private boolean solenoidEnabled;
    private double waitTime = 1e10;

    public MoveArmPiston(boolean solenoidEnabled) {
//        requires(TechnoTitan.arm);
        this.solenoidEnabled = solenoidEnabled;
    }

    @Override
    protected void initialize() {
//        boolean up = TechnoTitan.arm.isUp();
//        if (up == solenoidEnabled) waitTime = 0;
//        else if (up && !solenoidEnabled) waitTime = 1;
//        else if (!up && solenoidEnabled) waitTime = 1;

//        TechnoTitan.arm.setArmSolenoid(solenoidEnabled);
    }

    @Override
    protected boolean isFinished() {
        return timeSinceInitialized() > waitTime;
    }


}
