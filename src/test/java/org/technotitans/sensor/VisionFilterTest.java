 package org.technotitans.sensor;

 import com.kauailabs.navx.frc.AHRS;
 import edu.wpi.first.wpilibj.command.Scheduler;
 import edu.wpi.first.wpilibj.interfaces.Gyro;
 import frc.robot.OI;
 import frc.robot.TechnoTitan;
 import frc.robot.sensors.Encoder;
 import frc.robot.sensors.TimeOfFlight;
 import frc.robot.sensors.vision.VisionKalmanFilter;
 import frc.robot.sensors.vision.VisionSensor;
 import frc.robot.subsystems.DriveTrain;
 import org.junit.Before;
 import org.junit.Ignore;
 import org.junit.Test;

 import static junit.framework.TestCase.*;
 import static org.mockito.Mockito.*;

 public class VisionFilterTest {
     private VisionSensor mockVision;
     private AHRS mockNavX;
     private Encoder leftMock, rightMock;
     private TimeOfFlight tfDistance;

     @Before
     public void setUp() {
         mockVision = mock(VisionSensor.class);
         TechnoTitan.vision = mockVision;

         mockNavX = mock(AHRS.class);
         leftMock = mock(Encoder.class);
         rightMock = mock(Encoder.class);
         DriveTrain driveTrainMock = mock(DriveTrain.class);

         tfDistance = mock(TimeOfFlight.class);

         when(driveTrainMock.getLeftEncoder()).thenReturn(leftMock);
         when(driveTrainMock.getRightEncoder()).thenReturn(rightMock);

         TechnoTitan.navx = mockNavX;
         TechnoTitan.centralGyro = mock(Gyro.class);
         TechnoTitan.drive = driveTrainMock;
         TechnoTitan.tfDistance = tfDistance;

         TechnoTitan.oi = mock(OI.class);
     }

     @Test

     public void visionFilterShouldGetDistances() {
         final double dist = 60.0;
         // Note that filter.start assumes targets can be seen
         when(mockVision.getYDistance()).thenReturn(dist);

         VisionKalmanFilter filter = new VisionKalmanFilter();
         filter.initialize();

         assertEquals(-dist, filter.getSensorData().getY());
         verify(mockVision, times(1)).getYDistance();
     }

     @Test
     @Ignore
     public void filterShouldUseEncodersToMakeUpForVision() {
         final double dist = 45.0;

         when(mockVision.getYDistance()).thenReturn(dist);

         VisionKalmanFilter filter = new VisionKalmanFilter();
         filter.initialize();

         // It then loses sight of the targets
         final double speed = 2.0;  // in/s
         final long time = 20; // ms
         when(leftMock.getSpeedInches()).thenReturn(speed);
         when(rightMock.getSpeedInches()).thenReturn(speed);
         when(mockVision.canSeeTargets()).thenReturn(false);
         when(mockVision.getYDistance()).thenReturn(-1.0);

         try {
             Thread.sleep(time);
         } catch (InterruptedException e) {
             fail("Thread interrupted");
         }

         filter.execute();
         assertEquals(-dist + speed * time / 1000, filter.getSensorData().getY(), 5e-3);
     }

     @Test
     public void filterShouldUseDistanceSensor() {
         final double dist = 45.0;

         when(mockVision.getYDistance()).thenReturn(dist);

         VisionKalmanFilter filter = new VisionKalmanFilter();
         filter.initialize();

         when(tfDistance.getDistance()).thenReturn(dist - 5);
         when(tfDistance.isValid()).thenReturn(true);

         when(mockVision.canSeeTargets()).thenReturn(false);
         when(mockVision.getYDistance()).thenReturn(-1.0);

         filter.execute();
         double y = filter.getSensorData().getY();
         assertTrue("Y value not combined with distance sensor correctly: " + y, -(dist - 2) < y && y < -(dist - 5));
     }
 }
