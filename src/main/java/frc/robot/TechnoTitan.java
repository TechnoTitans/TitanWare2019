/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motor.TalonSRX;
import frc.robot.sensors.LimitSwitch;
import frc.robot.sensors.NavXGyro;
import frc.robot.sensors.QuadEncoder;
import frc.robot.sensors.TimeOfFlight;
import frc.robot.sensors.gy521.Accel_GY521;
import frc.robot.sensors.vision.VisionSensor;
import frc.robot.subsystems.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class TechnoTitan extends TimedRobot {

  public static OI oi;
  public static DriveTrain drive;
//  public static Arm arm;
  public static Elevator elevator;
  public static Wrist wrist;
  public static AHRS navx;
  public static VisionSensor vision;
  public static TimeOfFlight tfDistance;
  public static Grabber grabber;
  public static Gyro centralGyro;


  private TalonSRX elevatorMotor, wristMotor;

  private static final boolean LEFT_REVERSE = false,
                               RIGHT_REVERSE = true;

  private static final double INCHES_PER_PULSE = 0.00570;

//  private static final int MVA_TAPS = 25;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    navx = new AHRS(SPI.Port.kMXP);
    navx.reset();

    vision = new VisionSensor();
    tfDistance = new TimeOfFlight();
    centralGyro = new NavXGyro(navx);



    // elevator setup
    // 0.0000194 in/pulse?
    elevatorMotor = new TalonSRX(BlinkyMap.ELEVATOR_MOTOR, true);
    elevatorMotor.setEncoder(new QuadEncoder(elevatorMotor, 0.00274, false)); // TODO: configure
    // moving down is positive
    wristMotor = new TalonSRX(BlinkyMap.WRIST_MOTOR, false);
    wristMotor.setEncoder(new QuadEncoder(wristMotor, 1, false));

    // MARK - accelerometer setup

    DigitalInput limitSwitchTop = new DigitalInput(RobotMap.LS_TOP);
    DigitalInput limitSwitchBottom = new DigitalInput(RobotMap.LS_BOT);


     
    elevator = new Elevator(elevatorMotor, new LimitSwitch(limitSwitchTop, true), new LimitSwitch(limitSwitchBottom, true));
//    arm = new Arm(elbow, wrist, elbowAngleSensor, wristAngleSensor);
    wrist = new Wrist(wristMotor);
    grabber = new Grabber(new TalonSRX(BlinkyMap.GRABBER_MOTOR, false), new TalonSRX(BlinkyMap.WRIST_MOTOR, false), new Solenoid(RobotMap.PCM_ADDR, RobotMap.HATCH_MECH_EXTEND_PISTON), new Solenoid(RobotMap.PCM_ADDR, RobotMap.HATCH_GRAB_PISTON));


    // Drivetrain setup
    TalonSRX leftETalonSRX = new TalonSRX(BlinkyMap.LEFT_TALON_E, LEFT_REVERSE),
             rightETalonSRX = new TalonSRX(BlinkyMap.RIGHT_TALON_E, RIGHT_REVERSE);
    leftETalonSRX.setEncoder(new QuadEncoder(leftETalonSRX, INCHES_PER_PULSE, true));
    rightETalonSRX.setEncoder(new QuadEncoder(rightETalonSRX, INCHES_PER_PULSE, true));

    TalonSRX leftFollow1 = new TalonSRX(BlinkyMap.LEFT_TALON_2, LEFT_REVERSE),
            leftFollow2 = new TalonSRX(BlinkyMap.LEFT_TALON_3, LEFT_REVERSE),
            rightFollow1 = new TalonSRX(BlinkyMap.RIGHT_TALON_2, RIGHT_REVERSE),
            rightFollow2 = new TalonSRX(BlinkyMap.RIGHT_TALON_3, RIGHT_REVERSE);

    leftFollow1.follow(leftETalonSRX);
    leftFollow2.follow(leftETalonSRX);
    rightFollow1.follow(rightETalonSRX);
    rightFollow2.follow(rightETalonSRX);

    leftETalonSRX.setupCurrentLimiting();
    rightETalonSRX.setupCurrentLimiting();
    leftFollow1.setupCurrentLimiting();
    leftFollow2.setupCurrentLimiting();
    rightFollow1.setupCurrentLimiting();
    rightFollow2.setupCurrentLimiting();

    drive = new TankDrive(leftETalonSRX, rightETalonSRX);
    oi = new OI(); // must initialize oi after drive because it requires it as a a subsystem

    drive.resetEncoders();

    vision.stopRecording();

    CameraServer.getInstance().startAutomaticCapture(0);
//    CameraServer.getInstance().startAutomaticCapture(1);

    Thread updateToF = new Thread(() -> {
      while (!Thread.interrupted()) {
        tfDistance.update();
        try {
          Thread.sleep(20);
        } catch(InterruptedException e) {}
      }
    });
    updateToF.setDaemon(true);
    updateToF.start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
      SmartDashboard.putNumber("Elevator Talon Voltage", elevatorMotor.getCurrent());
      SmartDashboard.putNumber("Wrist Talon Voltage", wristMotor.getCurrent());
    // MARK - smart dashboard things
//    SmartDashboard.putNumber("NavX Gyro", navx.getAngle());
    SmartDashboard.putNumber("Gyro Angle", centralGyro.getAngle());
    SmartDashboard.putNumber("Angle error", VisionSensor.getAngleTargetDiff());

    SmartDashboard.putNumber("Encoder left", drive.getLeftEncoder().getDistance());
    SmartDashboard.putNumber("Encoder right", drive.getRightEncoder().getDistance());

    SmartDashboard.putNumber("Wrist encoder", wrist.getPosition());
    SmartDashboard.putNumber("Elevator encoder", elevator.getPosition());

    SmartDashboard.putNumber("Wrist angle", wrist.getAngle());
    SmartDashboard.putNumber("Elevator height", elevator.getHeight());

    wristMotor.postEstimatedKf("Wrist");
    elevatorMotor.postEstimatedKf("Elevator");

    elevator.compensateEncoder();

//    elevator.collectData();

    SmartDashboard.putBoolean("Are sensors overriden", elevator.areSensorsOverridden());

    SmartDashboard.putNumber("TF Distance", tfDistance.getDistance());
    SmartDashboard.putBoolean("TF is valid?", tfDistance.isValid());



    if (oi.shouldResetCommands()) {
      // TODO Uncomment out the remove all
      SmartDashboard.putNumber("Resetting", Math.random());
      Scheduler.getInstance().removeAll();
    }

    if (oi.shouldResetEncoders()) {
      wrist.resetEncoder();
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    // todo reset elevator pid stuff
//    TechnoTitan.arm.elbowController.reset();
//    TechnoTitan.arm.wristController.reset();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    vision.startRecording();
    VisionSensor.initGyro();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putNumber("Time left in sandstorm", DriverStation.getInstance().getMatchTime());
  }

  @Override
  public void teleopInit() {
    vision.startRecording();
    VisionSensor.initGyro();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    if (DriverStation.getInstance().getMatchTime() < 5) {
      vision.stopRecording();
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
