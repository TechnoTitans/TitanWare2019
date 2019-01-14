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
import frc.robot.movements.ControlDriveTrain;
import frc.robot.movements.ControlDriveTrainStraight;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private Joystick left, right, aux;
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

    public void initialize() {
        left = new Joystick(RobotMap.LEFT_JOYSTICK);
        right = new Joystick(RobotMap.RIGHT_JOYSTICK);
        aux = new Joystick(RobotMap.AUX_JOYSTICK);
        Button driveTriggerLeft = new JoystickButton(left, 0);
        driveTriggerLeft.whenPressed(new ControlDriveTrainStraight(this));
        driveTriggerLeft.whenReleased(new ControlDriveTrain(this));
    }

    public double getLeft() {
        return -left.getY();
    }

    public double getRight() {
        return -right.getY();
    }

    public double getElbowMove() {
        return -aux.getY();
    }

    public double getWristMove() {
        return aux.getPOV(0);
    }
}
