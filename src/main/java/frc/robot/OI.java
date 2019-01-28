/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.movements.ArmPosition;
import frc.robot.movements.AutoAlign;
import frc.robot.movements.ControlArm;
import frc.robot.movements.ControlArmPID;
import frc.robot.movements.ControlDriveTrainStraight;
import frc.robot.movements.ForwardAlign;
import frc.robot.subsystems.Arm;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private Joystick left, right, aux1, aux2;
    public static final double percentDeadbandThreshold = 0.01;


    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a
    //// joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

    private static final int TOGGLE_ARM_UP_BTN = 0;

    public OI() {
        initialize();
    }

    private void initialize() {
        left = new Joystick(RobotMap.LEFT_JOYSTICK);
        right = new Joystick(RobotMap.RIGHT_JOYSTICK);
        aux1 = new Joystick(RobotMap.AUX_JOYSTICK_1);
        aux2 = new Joystick(RobotMap.AUX_JOYSTICK_2);

        Button driveTriggerLeft = new JoystickButton(left, 1);
        Button btnArmLevel1 = new JoystickButton(aux2, 1);
        Button btnArmLevel2 = new JoystickButton(aux2, 2);
        Button btnArmLevel3 = new JoystickButton(aux2, 3);

        Button autoAlign = new JoystickButton(right, 3);
        Button forwardAlign = new JoystickButton(aux1, 1);

        driveTriggerLeft.whileHeld(new ControlDriveTrainStraight());

        // arm controls
        btnArmLevel1.whenPressed(new ControlArmPID(ArmPosition.ROCKET_LEVEL_1));
        btnArmLevel2.whenPressed(new ControlArmPID(ArmPosition.ROCKET_LEVEL_2));
        btnArmLevel3.whenPressed(new ControlArmPID(ArmPosition.ROCKET_LEVEL_3));

        AutoAlign align = new AutoAlign();
        autoAlign.toggleWhenPressed(align);

        forwardAlign.whenPressed(new ForwardAlign(ArmPosition.ROCKET_LEVEL_1, 60, 0.5));
    }

    public double clampInput(double input) {
        if (input > percentDeadbandThreshold || input < -percentDeadbandThreshold) {
            return input;
        } else {
            return 0;
        }
    }

    public double getLeft() {
        return -left.getRawAxis(1);
        // return clampInput(-left.getY());
    }

    public double getRight() {
        return clampInput(-right.getRawAxis(1));
    }

    public double getElbowMove() {
        return clampInput(-aux1.getY());
    }

    public double getWristMove() {
        return clampInput(-aux2.getY());
    }

    public boolean toggleArmUp() {
        return aux1.getRawButtonPressed(TOGGLE_ARM_UP_BTN);
    }
}
