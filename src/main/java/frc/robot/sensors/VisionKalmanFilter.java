package frc.robot.sensors;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.TechnoTitan;

import java.util.Arrays;

public class VisionKalmanFilter {
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
    }

    private double x;
    private double y;

    private NavXGyro gyro;


    private double angle;

    private static final double MAX_DRIVE_SPEED = 20;
    private static final double ROBOT_RADIUS = 18;


    private Matrix covMatrix;

    private Timer lastTime;

    public VisionKalmanFilter() {
        gyro = new NavXGyro();
    }

    public void start() {
        x = TechnoTitan.vision.getXOffset();
        y = -TechnoTitan.vision.getYDistance();
        angle = Math.toRadians(TechnoTitan.vision.getSkew());
        covMatrix = new Matrix(new double[][] {
                {25, 0, 0},
                {0, 9, 0},
                {0, 0, 25}
        });

        lastTime = new Timer();
        gyro.reset();
    }

    public void update() {
        double dt = lastTime.get();

        double lSpeed = TechnoTitan.drive.getLeftEncoder().getSpeedInches(),
                rSpeed = TechnoTitan.drive.getRightEncoder().getSpeedInches();
        if ((Math.abs(lSpeed) < 0.01 && Math.abs(rSpeed) < 0.01) || dt < 0.001) {
            // almost no change
            return;
        }
        double averageSpeed = (lSpeed + rSpeed) / 2;
        double predictedX = x + averageSpeed * Math.sin(angle) * dt;
        double predictedY = y + averageSpeed * Math.cos(angle) * dt;
        double angleChange = Math.toRadians(gyro.getRate()) * dt;
        double predictedAngle = angle + angleChange;
        /* (x, y, angle)
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

        if (TechnoTitan.vision.canSeeTargets()) {
            double xResidual = TechnoTitan.vision.getXOffset() - predictedX;
            double yResidual = -TechnoTitan.vision.getYDistance() - predictedY;
            double angleResidual = Math.toRadians(TechnoTitan.vision.getSkew()) - predictedAngle;


            Matrix H = new Matrix(new double[][]{
                    {1, 0, 0},
                    {0, 1, 0},
                    {0, 0, 1}
            });

            Matrix R = new Matrix(new double[][]{
                    {25, 0, 0},
                    {0, 9, 0},
                    {0, 0, 25}
            });

//            Matrix S = H.multiply(covMatrix).multiply(H.transpose());
//            S.add(R);
            // Equivalent to (since H is identity for now)
            Matrix S = R;
            S.add(covMatrix);

            Matrix K = covMatrix.multiply(H.transpose()).multiply(S.invert());
            double[] gains = K.multiply(new double[] {xResidual, yResidual, angleResidual});
            predictedX += gains[0];
            predictedY += gains[1];
            predictedAngle += gains[2];

            Matrix eye = new Matrix(new double[][] {
                    {1, 0, 0}, {0, 1, 0}, {0, 0, 1}
            });

            eye.subtract(K.multiply(H));
            covMatrix = eye.multiply(covMatrix);
        }

        this.x = predictedX;
        this.y = predictedY;
        this.angle = predictedAngle;

        lastTime.reset();
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
}
