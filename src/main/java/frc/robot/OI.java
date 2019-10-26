/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.movements.*;
import frc.robot.movements.elevator.ControlElevatorWristTeleop;
import frc.robot.movements.elevator.ElevatorPosition;
import frc.robot.movements.elevator.MoveElevatorToPosition;
import frc.robot.sensors.vision.VisionKalmanFilter;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private Joystick left, right;
    private XboxController xbox;
    private static final double percentDeadbandThreshold = 0.1;


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

    private Button btnGrabberExpel,
                    btnGrabberIntake,
                    btnOverrideSensors,
                    btnResetCommands,
                    btnDriveSlow,
                    btnEmergencyResetSensors;

//    private Btn btnMoveTargetLeft, btnMoveTargetRight;

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

    public boolean isXboxOnRocket() {
        return xbox.getTriggerAxis(GenericHID.Hand.kLeft) < 0.5;
    }

    private void initialize() {
        left = new Joystick(RobotMap.LEFT_JOYSTICK);
        right = new Joystick(RobotMap.RIGHT_JOYSTICK);
        xbox = new XboxController(RobotMap.AUX_JOYSTICK_1);
        Button climb = new Btn(left, 11);
        Button driveTriggerLeft = new JoystickButton(left, 1);
        Button autoAlign = new Btn(left, 3);
        Button forwardAlign = new Btn(left, 2);
        Button forwardAlignHit = new Btn(left, 5);
        Button autoAlignAngle = new Btn(left, 4);
        Button launchButton = new Btn(left, 7);
        Button launchButton2 = new Btn(left, 6);
        Button expelGrabberAndBackup = new Btn(right, 5);
        Button toggleBallHatchMode = new Button() {
            @Override
            public boolean get() {
                return xbox.getBumper(GenericHID.Hand.kLeft) && isXboxOnRocket();
            }
        };

        Button toggleGrabHatch = new Button() {
            @Override
            public boolean get() {
                return xbox.getBumper(GenericHID.Hand.kRight);
            }
        };
//
//        btnMoveTargetLeft = new Btn(left, 4);
//        btnMoveTargetRight = new Btn(left, 5);

        Button btnGrabHatch = new Btn(right, 4);
        btnGrabberIntake = new Btn(right, 2);
        btnGrabberExpel = new Btn(right, 3);

        Button btnRocketBall1 = new Button() {
            @Override
            public boolean get() {
                return xbox.getAButton() && isXboxOnRocket();
            }
        };
        Button defense = new Button() {
            @Override
            public boolean get() {
                return xbox.getBumper(GenericHID.Hand.kRight) && isXboxOnRocket();
            }
        };
        Button btnRocketBall2 = new Button() {
            @Override
            public boolean get() {
                return xbox.getXButton() && isXboxOnRocket();
            }
        };

        Button btnRocketBallCargo = new Button() {
            @Override
            public boolean get() {
                return xbox.getBButton() && isXboxOnRocket();
            }
        };

        Button btnRocketBallPickup = new Button() {
            @Override
            public boolean get() {
                return xbox.getTriggerAxis(GenericHID.Hand.kRight) > 0.5;
            }
        };

        Button btnRocketBall3 = new Button() {
            @Override
            public boolean get() { return xbox.getYButton() && isXboxOnRocket();
            }
        };

        Button btnHatch1 = new Button() {
            @Override
            public boolean get() {
                return xbox.getAButton() && !isXboxOnRocket();
            }
        };
        Button btnHatch2 = new Button() {
            @Override
            public boolean get() { return xbox.getXButton() && !isXboxOnRocket();
            }
        };

        Button btnStow = new Button() {
            @Override
            public boolean get() {
                return xbox.getBButton() && !isXboxOnRocket();
            }
        };

        Button btnHatch3 = new Button() {
            @Override
            public boolean get() {
                return xbox.getYButton() && !isXboxOnRocket();
            }
        };
        btnOverrideSensors = new Button() {
            @Override
            public boolean get() {
                return xbox.getStickButtonPressed(GenericHID.Hand.kRight) || xbox.getStickButtonPressed(GenericHID.Hand.kLeft);
            }
        };

        btnResetCommands = new Button() {
            @Override
            public boolean get() {
                return xbox.getStartButtonPressed() || xbox.getBackButtonPressed();  // todo: see if these work
            }
        };


//        driveTriggerLeft.whileHeld(new ControlDriveTrainStraight());

        // arm controls
        btnRocketBall1.whenPressed(new MoveElevatorToPosition(ElevatorPosition.ROCKET_LEVEL_1_BALL));
        btnRocketBall2.whenPressed(new MoveElevatorToPosition((ElevatorPosition.ROCKET_LEVEL_2_BALL)));
        btnRocketBall3.whenPressed(new MoveElevatorToPosition(ElevatorPosition.ROCKET_LEVEL_3_BALL));
        btnRocketBallCargo.whenPressed(new MoveElevatorToPosition((ElevatorPosition.CARGO_SHIP_BALL)));
        btnRocketBallPickup.whenPressed(new MoveElevatorToPosition((ElevatorPosition.BALL_PICKUP)));

        btnHatch1.whenPressed(new MoveElevatorToPosition((ElevatorPosition.LOW_HATCH)));
        btnHatch2.whenPressed(new MoveElevatorToPosition((ElevatorPosition.ROCKET_LEVEL_2_HATCH)));
        btnHatch3.whenPressed(new MoveElevatorToPosition(ElevatorPosition.ROCKET_LEVEL_3_HATCH));
        defense.whenPressed(new MoveElevatorToPosition(ElevatorPosition.DEFENSE));
        btnStow.whenPressed(new MoveElevatorToPosition((ElevatorPosition.STOW_POSITION)));


        VisionKalmanFilter visionFilter = new VisionKalmanFilter();
//        autoAlign.toggleWhenPressed(new AutoAlign(0.8, 20, visionFilter));
//        autoAlign.toggleWhenPressed(new AutoAlignAngle());
        autoAlign.whileHeld(new AutoAlignLine());
        autoAlign.whenReleased(new AutoAlignLineTotal());
//        forwardAlign.whenPressed(new ForwardAlign(ArmPosition.LOW_HATCH, 60, 0.5));
        forwardAlign.whileHeld(new ControlDriveTrainStraight());
        forwardAlignHit.toggleWhenPressed(new ForwardAlignLine());
        autoAlignAngle.toggleWhenPressed(new AutoAlignAngle());

        launchButton.whenPressed(new Launch(true));
        launchButton2.whenPressed(new Yeet());
        expelGrabberAndBackup.whenPressed(new ReleaseHatch());

        toggleBallHatchMode.whenPressed(new InstantCommand(TechnoTitan.grabber, () -> TechnoTitan.grabber.toggleBallHatchMode()));
//        toggleGrabHatch.whenPressed(new InstantCommand(TechnoTitan.grabber, () -> TechnoTitan.grabber.intakeBall()));
        btnGrabHatch.whenPressed(new GrabHatch());
        climb.whenPressed(new InstantCommand(TechnoTitan.climber, () -> TechnoTitan.climber.toggle()));
        btnOverrideSensors.whenPressed(new ControlElevatorWristTeleop());
    }

    private double clampInput(double input) {
        if (input > percentDeadbandThreshold || input < -percentDeadbandThreshold) {
            return input;
        } else {
            return 0;
        }
    }

    public double getLeft() {
        return clampInput(-left.getRawAxis(1));
    }

    public boolean getSlowdown() {
        return left.getRawButton(1);
    }

    public double getRight() {
        return clampInput(-right.getRawAxis(1));
    }

    public double getElevatorMove() {
        return -clampInput(xbox.getY(GenericHID.Hand.kLeft) * 2);
    }

    public double getWristMove() {
//        return -clampInput(xbox.getY(GenericHID.Hand.kRight) * 2);
        return clampInput(xbox.getY(GenericHID.Hand.kRight) * 2);
    }

    public boolean shouldExpelGrabber() {
        return btnGrabberExpel.get();
    }

    public boolean shouldIntakeGrabber() { return btnGrabberIntake.get();
    }

    public boolean shouldToggleOverrideSensors() {
        return false;
    }

    public boolean shouldResetCommands() {
        return xbox.getBackButtonPressed();
    }

    public boolean shouldResetEncoders() {
        return xbox.getStartButtonPressed();
    }

    public boolean getMoveTargetLeft() { return false; } //btnMoveTargetLeft.isPressed(); }

    public boolean getMoveTargetRight() { return false; } //btnMoveTargetRight.isPressed(); }
}
