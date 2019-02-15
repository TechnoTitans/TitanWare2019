package frc.robot.sensors.vision;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.*;
import org.opencv.imgproc.*;

@SuppressWarnings("WeakerAccess")
public class LineDetector {

    // MARK - camera config
    private static final int kCameraWidth = 640,
                             kCameraHeight = 480;
    private final UsbCamera camera;

    // MARK - OpenCV Config
    private static final double kBackgroundThreshold = 50;
    private static final int kKernelSize = 3;
    private static final double kGaussianSigma = 20.0;
    private static final double kCannyLowerThreshold = -999;
    private static final double kCannyUpperThreshold = kCannyLowerThreshold * 3;

    // MARK - OpenCV obj
    private CvSink cvSink;
    private CvSource outputStream;
    private Mat sourceMat, outputMat;

    // MARK - Instance stuff
    private double currentSkew;


    public LineDetector(UsbCamera camera) {
        this.camera = camera;
    }

    public void init() {
        // TODO DO we need this usb camera stuff, or is there a central camera class that handles this...
        // also does this interfere with the pi asking for frames from the camera?
        // for now offload this to the instantiator
//        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(kCameraWidth, kCameraHeight);

        cvSink = CameraServer.getInstance().getVideo();
        outputStream = CameraServer.getInstance().putVideo("LineDetector Stream", 640, 480);

        sourceMat = new Mat();
        outputMat = new Mat();
    }

    private void processImage() {
        Mat result = new Mat();
        cvSink.grabFrame(result);
        Imgproc.cvtColor(result, result, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(result, result, new Size(kKernelSize, kKernelSize), kGaussianSigma);
        Imgproc.Canny(result, result, kCannyLowerThreshold, kCannyUpperThreshold, kKernelSize);
            Imgproc.contourArea(result);
//       outputStream.putFrame(outputMat);
    }

    public void update() {
        this.processImage();
    }

    private void calculateSkew() {
        double newSkew  = 0.0;
        // calculate the skew here
        this.currentSkew = newSkew;
    }

    public double getSkew() {
        return this.currentSkew;
    }

    public boolean isLineInSight() {
        Mat result = new Mat();
        Imgproc.threshold(sourceMat, result, kBackgroundThreshold, 255, Imgproc.THRESH_BINARY);
        return false;
    }

}
