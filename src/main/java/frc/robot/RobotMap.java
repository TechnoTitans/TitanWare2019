/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
@SuppressWarnings("WeakerAccess")
public class RobotMap {
    // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static final int LEFT_JOYSTICK = 0;
  public static final int RIGHT_JOYSTICK = 1;
  public static final int AUX_JOYSTICK_1 = 2,
                          AUX_JOYSTICK_2 = 3;


//  public static final int ELBOW_MOTOR = 3;

  // Talons with encoders
  public static final int LEFT_TALON_E = 5, RIGHT_TALON_E = 8;

  // Other talons
  public static final int LEFT_TALON_2 = 6, LEFT_TALON_3 = 4, RIGHT_TALON_2 = 9, RIGHT_TALON_3 = 7;

  // address of the accelerometer
  public static final int ELBOW_ANGLE_ADDR = 0x69;
  public static final int WRIST_ANGLE_ADDR = 0x68;  // default

  public static final int PCM_ADDR = 15;
//  public static final int ARM_PISTON = 0;
    public static final int HATCH_MECH_EXTEND_PISTON = 0;
    public static final int HATCH_GRAB_PISTON = 1;
    public static final int CLIMB_PISTON = 2;
    public static final int WRIST_MOTOR = 1;
    public static final int GRABBER_MOTOR = 2;
    public static final int ELEVATOR_MOTOR = 3;

    public static final int LS_BOT = 0;
    public static final int LS_TOP = 1;
}
