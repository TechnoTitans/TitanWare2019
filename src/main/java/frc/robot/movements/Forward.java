// Do not make this a negative

package frc.robot.movements;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TechnoTitan;


public class Forward extends Command {
    private double distance;
    private double desiredSpeed;
    private double startSpeed = 0.2;
    // private double difference = desiredSpeed - startSpeed;
    private double speed = startSpeed;
    private double speedPerDistance = 0.05;
    private boolean accelerate = true;
    private boolean decelerate = false;
    private final double kP = 0.05; // error for angle
    private double previousEncoder = 0;

    private Timer minTime;

    private double encDiff;
    private double lastEnc;

    private boolean resetGyro;

    public Forward(double forwardNum, double speed, double acceleration, boolean resetGyro) {
        requires(TechnoTitan.drive);
        this.distance = forwardNum;
        this.desiredSpeed = speed;
        this.speedPerDistance = acceleration;
        this.resetGyro = resetGyro;
        minTime = new Timer();
    }

    public Forward(double forwardNum, double speed) {
        this(forwardNum, speed, 0, true);
        this.setInterruptible(true);
	}

    public Forward(double forwardNum, double speed, boolean resetGyro) {
        this(forwardNum, speed, 0, resetGyro);
    }
	@Override
    protected void initialize() {
        if (resetGyro) TechnoTitan.gyro.reset();
        TechnoTitan.drive.resetEncoders();
        SmartDashboard.putNumber("Started forward", distance);
        SmartDashboard.putBoolean("Is finished?", isFinished());
        minTime.start();

    }

    private double distanceToAccelerate = 0; //number of ticks to increase to maximum speed
    @Override
    protected void execute() {
        speed = desiredSpeed;
        /*if (speed >= desiredSpeed){
            accelerate = false;
        }

        double delta = TechnoTitan.drive.getLeftEncoder().getDistance() - previousEncoder;
        if (distance - TechnoTitan.drive.getLeftEncoder().getDistance() <= distanceToAccelerate){
            decelerate = true;
        }

        if (decelerate) {
            speed -= speedPerDistance * delta;
        } else if (accelerate){
            speed += speedPerDistance * delta;
            distanceToAccelerate += delta;
        }
        previousEncoder = TechnoTitan.drive.getLeftEncoder().getDistance();*/
        double error = TechnoTitan.gyro.getAngle();
        error *= kP;
        TechnoTitan.drive.set(speed - error, speed + error);
    }

    @Override
    protected boolean isFinished() {
        double speed = TechnoTitan.drive.getLeftEncoder().getSpeed();
        SmartDashboard.putNumber("Encoder speed", speed);
        return minTime.get() > 0.2 && (TechnoTitan.drive.getLeftEncoder().getDistance() > distance ||
              TechnoTitan.drive.getRightEncoder().getDistance() > distance || speed < 0.5);
    }

    @Override
    protected void end() {
        TechnoTitan.drive.stop();
        decelerate = false;
        accelerate = true;
    }

    @Override
    protected void interrupted() {
      end();
    }

    
}
