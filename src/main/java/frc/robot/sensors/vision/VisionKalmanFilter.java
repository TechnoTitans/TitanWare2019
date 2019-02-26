package frc.robot.sensors.vision;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.TechnoTitan;
import frc.robot.sensors.TitanGyro;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Queue;

public class VisionKalmanFilter {
    private static final int LAG_FRAMES = 5;

    private static class Matrix {
        private double[][] data;
        private int cols, rows;

        Matrix(double[][] data) {
            this.data = data;
            this.rows = data.length;
            this.cols = (rows > 0 ? data[0].length : 0);
        }

        Matrix(int rows, int cols) {
            this.cols = cols;
            this.rows = rows;
            data = new double[rows][cols];
        }

        Matrix multiply(Matrix other) {
            double[][] newData = new double[rows][other.cols];
            if (cols != other.rows) throw new IllegalArgumentException("Matrix shapes not compatible");
            for (int i = 0; i < rows; ++i) {
                for (int j = 0;  j < other.cols; ++j) {
                    double result = 0;
                    for (int k = 0; k < cols; ++k) {
                        result += data[i][k] * other.data[k][j];
                    }
                    newData[i][j] = result;
                }
            }
            return new Matrix(newData);
        }

        Matrix transpose() {
            double[][] result = new double[cols][rows];
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    result[j][i] = data[i][j];
                }
            }
            return new Matrix(result);
        }

        void add(Matrix other) {
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    data[i][j] += other.data[i][j];
                }
            }
        }

        void subtract(Matrix other) {
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    data[i][j] -= other.data[i][j];
                }
            }
        }

        double[] multiply(double[] other) {
            double[] result = new double[rows];
            if (other.length != cols) throw new IllegalArgumentException("Not enough to multiply");
            for (int i = 0; i < rows; ++i) {
                double t = 0;
                for (int j = 0; j < cols; ++j) {
                    t += other[i] * data[i][j];
                }
                result[i] = t;
            }
            return result;
        }

        // Methods from: https://www.sanfoundry.com/java-program-find-inverse-matrix/
        Matrix invert() {
            double[][] a = data;
            int n = rows;
            double x[][] = new double[n][n];
            double b[][] = new double[n][n];
            int index[] = new int[n];
            for (int i=0; i<n; ++i)
                b[i][i] = 1;

            // Transform the matrix into an upper triangle
            gaussian(a, index);

            // Update the matrix b[i][j] with the ratios stored
            for (int i=0; i<n-1; ++i)
                for (int j=i+1; j<n; ++j)
                    for (int k=0; k<n; ++k)
                        b[index[j]][k]
                                -= a[index[j]][i]*b[index[i]][k];

            // Perform backward substitutions
            for (int i=0; i<n; ++i)
            {
                x[n-1][i] = b[index[n-1]][i]/a[index[n-1]][n-1];
                for (int j=n-2; j>=0; --j)
                {
                    x[j][i] = b[index[j]][i];
                    for (int k=j+1; k<n; ++k)
                    {
                        x[j][i] -= a[index[j]][k]*x[k][i];
                    }
                    x[j][i] /= a[index[j]][j];
                }
            }
            return new Matrix(x);
        }

        // Method to carry out the partial-pivoting Gaussian
        // elimination.  Here index[] stores pivoting order.

        private static void gaussian(double a[][], int index[]) {
            int n = index.length;
            double c[] = new double[n];

            // Initialize the index
            for (int i=0; i<n; ++i)
                index[i] = i;

            // Find the rescaling factors, one from each row
            for (int i=0; i<n; ++i)
            {
                double c1 = 0;
                for (int j=0; j<n; ++j)
                {
                    double c0 = Math.abs(a[i][j]);
                    if (c0 > c1) c1 = c0;
                }
                c[i] = c1;
            }

            // Search the pivoting element from each column
            int k = 0;
            for (int j=0; j<n-1; ++j)
            {
                double pi1 = 0;
                for (int i=j; i<n; ++i)
                {
                    double pi0 = Math.abs(a[index[i]][j]);
                    pi0 /= c[index[i]];
                    if (pi0 > pi1)
                    {
                        pi1 = pi0;
                        k = i;
                    }
                }

                // Interchange rows according to the pivoting order
                int itmp = index[j];
                index[j] = index[k];
                index[k] = itmp;
                for (int i=j+1; i<n; ++i)
                {
                    double pj = a[index[i]][j]/a[index[j]][j];

                    // Record pivoting ratios below the diagonal
                    a[index[i]][j] = pj;

                    // Modify other elements accordingly
                    for (int l=j+1; l<n; ++l)
                        a[index[i]][l] -= pj*a[index[j]][l];
                }
            }
        }

        public String toString() {
            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < rows; ++i) {
                builder.append(Arrays.toString(data[i]));
                builder.append("\n");
            }
            return builder.toString();
        }

        public Matrix copy() {
            Matrix newMatrix = new Matrix(rows, cols);
            for (int i = 0; i < rows; ++i) {
                System.arraycopy(data[i], 0, newMatrix.data[i], 0, cols);
            }
            return newMatrix;
        }
    }

    private double x;
    private double y;

    private TitanGyro gyro;


    private double angle;
    private double prevGyroAngle = 0;

    private Matrix covMatrix;

    private Timer lastTime;

    public VisionKalmanFilter() {
        lastTime = new Timer();
//        gyro = new NavXGyro();
        gyro = new TitanGyro(TechnoTitan.centralGyro);
        visionLagBuffer = new ArrayDeque<>();
        visionPositionInfo = new VisionPositionInfo(0, 0, 0);
    }

    private static class SensorData {
        private final double encoderLeftSpeed;
        private final double encoderRightSpeed;
        private final double angleChange;
        private final double distance;
        private final double dt;

        SensorData(double encoderLeftSpeed, double encoderRightSpeed, double angleChange, double distance, double dt) {
            this.encoderLeftSpeed = encoderLeftSpeed;
            this.encoderRightSpeed = encoderRightSpeed;
            this.angleChange = angleChange;
            this.distance = distance;
            this.dt = dt;
        }

        double getEncoderLeftSpeed() {
            return encoderLeftSpeed;
        }

        double getEncoderRightSpeed() {
            return encoderRightSpeed;
        }

        double getAngleChange() {
            return angleChange;
        }

        double getDistance() {
            return distance;
        }

        double getDt() {
            return dt;
        }
    }

    public static class VisionPositionInfo {
        private double x, y, angle;
        private Matrix covMatrix;

        public VisionPositionInfo(double x, double y, double angle) {
            this.x = x;
            this.y = y;
            this.angle = angle;
            covMatrix = new Matrix(new double[][] {
                    {80, 0, 90},
                    {0, 9, 0},
                    {90, 0, 100}
            });
        }

        public VisionPositionInfo copy() {
            VisionPositionInfo copy = new VisionPositionInfo(x, y, angle);
            copy.covMatrix = covMatrix.copy();
            return copy;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getAngle() {
            return angle;
        }

        /**
         * Interpolates all non-vision data
         * @param sensors
         */
        void interpolateSensorData(SensorData sensors) {
            double lSpeed = sensors.getEncoderLeftSpeed(),
                    rSpeed = sensors.getEncoderRightSpeed();
            double averageSpeed = (lSpeed + rSpeed) / 2;
            double dt = sensors.getDt();
            x += averageSpeed * Math.sin(angle) * dt;
            y += averageSpeed * Math.cos(angle) * dt;
            double angleChange = sensors.getAngleChange();
            angle += angleChange;

            /* (x, y, angle, lSpeed, rSpeed), S = 2*averageSpeed
            F =
            1, 0, averageSpeed*dt*cos(angle)
            0, 1, -averageSpeed*dt*sin(angle)
            0, 0, 1
             */
            Matrix F = new Matrix(new double[][] {
                    {1, 0, averageSpeed * Math.cos(angle) * dt},
                    {0, 1, -averageSpeed * Math.sin(angle) * dt},
                    {0, 0, 1}
            });

            // noise
            final double variation = 2;
            Matrix Q = new Matrix(new double[][] {
                    {variation * dt, 0, 0},
                    {0, variation * dt, 0},
                    {0, 0, variation * dt}
            });

            covMatrix = F.multiply(covMatrix).multiply(F.transpose());
            covMatrix.add(Q);
        }

        void interpolateVisionData(double visionX, double visionY, double visionSkew) {
            double xResidual = visionX - x;
            double yResidual = -visionY - y;
            double angleResidual = Math.toRadians(visionSkew) - angle;

            Matrix H = new Matrix(new double[][]{
                    {1, 0, 0},
                    {0, 1, 0},
                    {0, 0, 1}
            });

            double angleVar = Math.max(30, -y * 2);
            Matrix R = new Matrix(new double[][]{
                    {angleVar * 0.8, 0, angleVar * 0.9},  // TODO: add value
                    {0, 9, 0},
                    {angleVar * 0.9, 0, angleVar}
            });

//            Matrix S = H.multiply(covMatrix).multiply(H.transpose());
//            S.add(R);
            // Equivalent to (since H is identity for now)
            Matrix S = R;
            S.add(covMatrix);

            Matrix K = covMatrix.multiply(H.transpose()).multiply(S.invert());
            double[] gains = K.multiply(new double[]{xResidual, yResidual, angleResidual});
            addGains(gains);

            Matrix eye = new Matrix(new double[][]{
                    {1, 0, 0}, {0, 1, 0}, {0, 0, 1}
            });

            eye.subtract(K.multiply(H));
            covMatrix = eye.multiply(covMatrix);
        }

        private void addGains(double[] gains) {
            x += gains[0];
            y += gains[1];
            angle += gains[2];
        }
    }

    private Queue<SensorData> visionLagBuffer;

    public VisionPositionInfo getSensorData() {
        VisionPositionInfo positionInfoCopy = visionPositionInfo.copy();
        for (SensorData sensors : visionLagBuffer) {
            positionInfoCopy.interpolateSensorData(sensors);
        }
        return positionInfoCopy;
    }

    private VisionPositionInfo visionPositionInfo;

    public void start() {
        visionLagBuffer.clear();
        x = TechnoTitan.vision.getXOffset();
        y = -TechnoTitan.vision.getYDistance();
        angle = Math.toRadians(TechnoTitan.vision.getSkew());
        visionPositionInfo = new VisionPositionInfo(x, y, angle);

        lastTime.reset();
        lastTime.start();
        gyro.reset();
    }

    public void updateSensorBuffer(double dt) {
        double lSpeed = TechnoTitan.drive.getLeftEncoder().getSpeedInches(),
                rSpeed = TechnoTitan.drive.getRightEncoder().getSpeedInches();
        visionLagBuffer.add(new SensorData(lSpeed, rSpeed, Math.toRadians(gyro.getAngle() - prevGyroAngle), -1, dt));
        prevGyroAngle = gyro.getAngle();
    }

    public void update() {
        double dt = lastTime.get();
        lastTime.reset();

        updateSensorBuffer(dt);
        if (visionLagBuffer.size() < LAG_FRAMES) return;
        SensorData sensors = visionLagBuffer.remove();
        visionPositionInfo.interpolateSensorData(sensors);

//        if (TechnoTitan.vision.canSeeTargets()) {
//            visionPositionInfo.interpolateVisionData(TechnoTitan.vision.getXOffset(), TechnoTitan.vision.getYDistance(), TechnoTitan.vision.getSkew());
//        }
    }
}
