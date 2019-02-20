package frc.robot.sensors.vision;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;

public class DriverCamera {
    private UsbCamera cam1, cam2;

    private VideoSink sink;
    public DriverCamera() {
        cam1 = CameraServer.getInstance().startAutomaticCapture(0);
        cam2 = CameraServer.getInstance().startAutomaticCapture(1);
        sink = CameraServer.getInstance().getServer();
    }
}
