package frc.robot;

public class BlinkyMap {
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final int AUX_JOYSTICK_1 = 2,
            AUX_JOYSTICK_2 = 3;


//    public static final int WRIST_MOTOR = 1;
//    public static final int ELBOW_MOTOR = 3;

    // Talons with encoders
    public static final double INCHES_PER_PULSE = 0.00475;
    public static final int LEFT_TALON_E = 5, RIGHT_TALON_E = 11;

    // Other talons
    public static final int LEFT_TALON_2 = 4, LEFT_TALON_3 = 3, RIGHT_TALON_2 = 12, RIGHT_TALON_3 = 13;
//    public static final int GRABBER_MOTOR = 2;

    // accelerometers
//  public static final int ELBOW_ACCEL_ADDR = 0x18;
//  public static final int WRIST_ACCEL_ADDR = 0x19; // todo setup the second accelerometer to use the secondary addr if its the same

    // address of the accelerometer
    public static final int ELBOW_ANGLE_ADDR = 0x69;
    public static final int WRIST_ANGLE_ADDR = 0x68;  // default

    public static final int PCM_ADDR = 15;
    public static final int ARM_PISTON = 0;
    public static final int HATCH_PANEL_PISTON = 1;


    public static final int WRIST_MOTOR = 9;
    public static final int GRABBER_MOTOR = 7;
    public static final int ELEVATOR_MOTOR = 10; // TODO calculate actual value

}
