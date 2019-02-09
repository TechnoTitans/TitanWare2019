/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.movements.*;
import frc.robot.movements.arm.ArmPosition;
import frc.robot.movements.arm.ControlArmPID;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    // TODO: change to one joystick
    private Joystick left, right, aux1, aux2;
    private static final double percentDeadbandThreshold = 0.01;


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

    // TODO: assign buttons
    private static final int TOGGLE_ARM_UP_BTN = 0;
    private static final int GRABBER_EXPEL_BTN = 0;
    private static final int GRABBER_INTAKE_BTN = 1;

    private Btn btnToggleArmUp,
                    btnGrabberExpel,
                    btnGrabberIntake,
                    btnOverrideSensors,
                    btnResetCommands;

    public OI() {
        initialize();
    }

    // Joystick button with some extended features
    private static class Btn extends JoystickButton {
        private GenericHID hid;
        private int buttonNumber;

        public Btn(GenericHID hid, int buttonNumber) {
            super(hid, buttonNumber);
            this.hid = hid;
            this.buttonNumber = buttonNumber;
        }

        public boolean isPressed() {
            return hid.getRawButtonPressed(buttonNumber);
        }

        public boolean isReleased() {
            return hid.getRawButtonReleased(buttonNumber);
        }

        public boolean isHeld() {
            return get();
        }
    }

    private void initialize() {
        left = new Joystick(RobotMap.LEFT_JOYSTICK);
        right = new Joystick(RobotMap.RIGHT_JOYSTICK);
        aux1 = new Joystick(RobotMap.AUX_JOYSTICK_1);
        aux2 = new Joystick(RobotMap.AUX_JOYSTICK_2);

        // TODO: assign buttons
        Button driveTriggerLeft = new JoystickButton(left, 1);

        btnGrabberExpel = new Btn(aux1, 1);
        Button btnRocketBall1 = new Btn(aux1, 2);
        Button btnRocketBall2 = new Btn(aux1, 3);
        Button btnRocketBall3 = new Btn(aux1, 4);
        Button btnRocketBallCargo = new Btn(aux1, 5);
        Button btnRocketBallPickup = new Btn(aux1, 6);

        btnGrabberIntake = new Btn(aux2, 1);
        Button btnHatch1 = new Btn(aux2, 2);
        Button btnHatch2 = new Btn(aux2, 3);
        Button btnHatch3 = new Btn(aux2, 4);
        btnToggleArmUp = new Btn(aux2, 5);
        btnOverrideSensors = new Btn(aux2, 6);
        btnResetCommands = new Btn(aux2, 7);

        Button autoAlign = new Btn(right, 2);
        Button forwardAlign = new Btn(right, 3);


        driveTriggerLeft.whileHeld(new ControlDriveTrainStraight());

        // arm controls
        btnRocketBall1.whenPressed(new ControlArmPID(ArmPosition.ROCKET_LEVEL_1_BALL));
        btnRocketBall2.whenPressed(new ControlArmPID(ArmPosition.ROCKET_LEVEL_2_BALL));
        btnRocketBall3.whenPressed(new ControlArmPID(ArmPosition.ROCKET_LEVEL_3_BALL));

        btnRocketBallCargo.whenPressed(new ControlArmPID(ArmPosition.CARGO_SHIP_BALL));
        btnRocketBallPickup.whenPressed(new ControlArmPID(ArmPosition.BALL_PICKUP));

        btnHatch1.whenPressed(new ControlArmPID(ArmPosition.LOW_HATCH));
        btnHatch2.whenPressed(new ControlArmPID(ArmPosition.ROCKET_LEVEL_2_HATCH));
        btnHatch3.whenPressed(new ControlArmPID(ArmPosition.ROCKET_LEVEL_3_HATCH));


        autoAlign.toggleWhenPressed(new AutoAlign(0.5, 20));

        forwardAlign.whenPressed(new ForwardAlign(ArmPosition.ROCKET_LEVEL_1_BALL, 60, 0.5));
    }

    public double clampInput(double input) {
        if (input > percentDeadbandThreshold || input < -percentDeadbandThreshold) {
            return input;
        } else {
            return 0;
        }
    }

    public double getLeft() {
        return clampInput(-left.getRawAxis(1));
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
        return btnToggleArmUp.isPressed();
    }

    public boolean shouldExpelGrabber() {
        return btnGrabberExpel.isHeld();
    }

    public boolean shouldIntakeGrabber() {
        return btnGrabberIntake.isHeld();
    }

    public boolean toggleOverrideSensors() {
        return btnOverrideSensors.isPressed();
    }

    public boolean shouldResetCommands() {
        return btnResetCommands.isPressed();
    }
}
