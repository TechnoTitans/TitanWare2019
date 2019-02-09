/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motor.TalonSRX;
import frc.robot.sensors.QuadEncoder;
import frc.robot.sensors.TimeOfFlight;
import frc.robot.sensors.gy521.Accel_GY521;
import frc.robot.sensors.vision.VisionSensor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.TankDrive;


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
  public static Arm arm;
  public static AHRS navx;
  public static VisionSensor vision;
  public static TimeOfFlight tfDistance;
  public static Grabber grabber;

  private Accel_GY521 elbowAngleSensor;
  private Accel_GY521 wristAngleSensor;

  private static final boolean LEFT_REVERSE = false,
                               RIGHT_REVERSE = true;

  private static final double INCHES_PER_PULSE = 0.0045;

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

    // Arm setup
    TalonSRX wrist = new TalonSRX(RobotMap.WRIST_MOTOR, false),
            elbow = new TalonSRX(RobotMap.ELBOW_MOTOR, false);


    // MARK - accelerometer setup

//    elbowAngleSensor = new Accel_LIS3DH(RobotMap.ELBOW_ACCEL_ADDR);
//    wristAngleSensor = new Accel_LIS3DH(RobotMap.WRIST_ACCEL_ADDR);

//    movingAverageFilter = LinearDigitalFilter.movingAverage(elbowAngleSensor, MVA_TAPS);
//    singlePoleIIRFilter = LinearDigitalFilter.singlePoleIIR(elbowAngleSensor, TIME_CONSTANT, 0.01);
    elbowAngleSensor = new Accel_GY521(RobotMap.ELBOW_ANGLE_ADDR, false);
    wristAngleSensor = new Accel_GY521(RobotMap.WRIST_ANGLE_ADDR, false);
    arm = new Arm(elbow, wrist, new Solenoid(RobotMap.PCM_ADDR, RobotMap.ARM_PISTON), elbowAngleSensor, wristAngleSensor);
    grabber = new Grabber(new TalonSRX(RobotMap.GRABBER_MOTOR, false));


    // Drivetrain setup
    TalonSRX leftETalonSRX = new TalonSRX(RobotMap.LEFT_TALON_E, LEFT_REVERSE),
             rightETalonSRX = new TalonSRX(RobotMap.RIGHT_TALON_E, RIGHT_REVERSE);
    leftETalonSRX.setEncoder(new QuadEncoder(leftETalonSRX, INCHES_PER_PULSE, true));
    rightETalonSRX.setEncoder(new QuadEncoder(rightETalonSRX, INCHES_PER_PULSE, true));

    TalonSRX leftFollow1 = new TalonSRX(RobotMap.LEFT_TALON_2, LEFT_REVERSE),
            leftFollow2 = new TalonSRX(RobotMap.LEFT_TALON_3, LEFT_REVERSE),
            rightFollow1 = new TalonSRX(RobotMap.RIGHT_TALON_2, RIGHT_REVERSE),
            rightFollow2 = new TalonSRX(RobotMap.RIGHT_TALON_3, RIGHT_REVERSE);

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
    oi = new OI(); // must initializae oi after drive because it requires it as a a subsystem

    drive.resetEncoders();
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
    // MARK - sensor updates
    elbowAngleSensor.update();


    // MARK - smart dashboard things
    SmartDashboard.putNumber("NavX Gyro", navx.getAngle());

    SmartDashboard.putBoolean("Accel Gyro Test Success", elbowAngleSensor.isConnected());
    SmartDashboard.putNumber("Elbow angle", elbowAngleSensor.getAngle());

    // time of flight
    SmartDashboard.putNumber("Encoder left", drive.getLeftEncoder().getDistance());
    SmartDashboard.putNumber("Encoder right", drive.getRightEncoder().getDistance());
    SmartDashboard.putNumber("TF Distance", tfDistance.getDistance());
    SmartDashboard.putBoolean("TF is valid?", tfDistance.isValid());
    try {
      tfDistance.update();
    } catch (UncleanStatusException e) {
      // serial port not started yet
      System.out.println("Warning: " + e);
    }
    SmartDashboard.putNumber("NavX Angle", navx.getAngle());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
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

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
