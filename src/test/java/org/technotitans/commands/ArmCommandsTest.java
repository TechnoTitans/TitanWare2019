// package org.technotitans.commands;
//
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.command.Scheduler;
// import frc.robot.OI;
// import frc.robot.TechnoTitan;
// import frc.robot.motor.Motor;
// import frc.robot.movements.arm.ArmPosition;
// import frc.robot.movements.arm.ControlArm;
// import frc.robot.movements.arm.MoveArmToPosition;
// import frc.robot.sensors.gy521.Accel_GY521;
// import frc.robot.subsystems.Arm;
// import org.junit.Before;
// import org.junit.BeforeClass;
// import org.junit.Ignore;
// import org.junit.Test;
//
// import java.lang.reflect.Field;
//
// import static junit.framework.TestCase.*;
// import static org.mockito.Mockito.*;
//
// @Deprecated
// public class ArmCommandsTest {
//     private static Motor elbowMotor, wristMotor;
//     private static Solenoid solenoid;
//     private static Accel_GY521 elbowSensor, wristSensor;
//
//     private static OI oi;
//
//     private abstract static class SimpleMotor implements Motor {
//         private double lastSet = 0;
//
//         @Override
//         public void set(double speed) {
//             lastSet = speed;
//         }
//
//         @Override
//         public double getPercentSpeed() {
//             return lastSet;
//         }
//     }
//
//     @BeforeClass
//     public static void setUpScheduler() {
//         Scheduler.getInstance().enable();
//
//         // hack to ensure driver station is "enabled"
//         DriverStation mockStation = mock(DriverStation.class);
//         when(mockStation.isEnabled()).thenReturn(true);
//         try {
//             Field f = DriverStation.class.getDeclaredField("instance");
//             f.setAccessible(true);
//             f.set(null, mockStation);
//         } catch (NoSuchFieldException | IllegalAccessException e) {
//             e.printStackTrace();
//         }
//
//         elbowMotor = mock(SimpleMotor.class);
//         wristMotor = mock(SimpleMotor.class);
//         doCallRealMethod().when(elbowMotor).set(anyDouble());
//         doCallRealMethod().when(wristMotor).set(anyDouble());
//         doCallRealMethod().when(elbowMotor).getPercentSpeed();
//         doCallRealMethod().when(wristMotor).getPercentSpeed();
//
//         solenoid = mock(Solenoid.class);
//         elbowSensor = mock(Accel_GY521.class);
//         wristSensor = mock(Accel_GY521.class);
//
//         TechnoTitan.arm = new Arm(elbowMotor, wristMotor, solenoid, elbowSensor, wristSensor);
//
//         oi = mock(OI.class);
//         TechnoTitan.oi = oi;
//     }
//
//     @Before
//     public void setUp() {
//         // Initial angles
//         when(elbowSensor.getAngle()).thenReturn(-71.7);
//         when(wristSensor.getAngle()).thenReturn(87.5);
//     }
//
//     @Test
//     @Ignore  // This test will fail while there is no default command
//     public void testDefaultCommandAdded() {
//         Scheduler.getInstance().run();
//         assertTrue(TechnoTitan.arm.getDefaultCommand().isRunning());
//     }
//
//     @Test
//     public void testRequirementsOfCommand() {
//         Command c = new MoveArmToPosition(ArmPosition.LOW_HATCH);
//         assertTrue(c.doesRequire(TechnoTitan.arm));
//     }
//
//     @Test
//     public void requirementsCancelOut() {
//         Command c = new ControlArm();
//         c.start();
//         Scheduler.getInstance().run();
//         assertTrue(c.isRunning());
//         assertEquals(c, TechnoTitan.arm.getCurrentCommand());
//         Command newC = new MoveArmToPosition(ArmPosition.LOW_HATCH);
//         newC.start();
//         Scheduler.getInstance().run();
//         assertFalse(c.isRunning());
//
//         assertEquals(newC, TechnoTitan.arm.getCurrentCommand());
//         Scheduler.getInstance().removeAll();
//         Scheduler.getInstance().run();
//     }
//
//     // TODO: implement test for PID
//
//     @Test
//     public void testJoystickControls() {
//         Command c = new ControlArm();
//         c.start();
//         Scheduler.getInstance().run();
//         assertTrue(c.isRunning());
//         assertEquals(c, TechnoTitan.arm.getCurrentCommand());
//         Scheduler.getInstance().run();
//         assertEquals(elbowMotor.getPercentSpeed(), 0, 1e-5);
//         assertEquals(wristMotor.getPercentSpeed(), 0, 1e-5);
//
//         final double joystickPress = 0.5;
//         when(oi.getElevatorMove()).thenReturn(joystickPress);
//
//         Scheduler.getInstance().run();
//         verify(oi, atLeast(2)).getElevatorMove();
//
//         // Filter constant should definitely be smaller than 0.3 or else we've done something very wrong
//         double lastSet = elbowMotor.getPercentSpeed();
//         assertTrue("Last elbow set value " + lastSet + " is not in range", 0 < lastSet && lastSet < joystickPress * 0.3);
//         assertEquals(wristMotor.getPercentSpeed(), 0.0, 1e-5);
//     }
// }
