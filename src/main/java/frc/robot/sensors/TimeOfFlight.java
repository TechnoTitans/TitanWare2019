package frc.robot.sensors;
import edu.wpi.first.wpilibj.I2C;

public class TimeOfFlight {
    I2C.Port port;
    int address;
    I2C timeOfFlight;
    byte[] expectedWrite = {0, 1, 0, 1, 0, 0, 1, 0};
    byte[] expectedRead = {0, 1, 0, 1, 0, 0, 1, 1};

    public TimeOfFlight(I2C.Port port, int address){
        this.address = address;
        this.port = port;
        timeOfFlight = new I2C (port, address);

    }

    public double getDistance(){
        return 0;
    }

    public boolean checkConnection(){
        return timeOfFlight.verifySensor(address, 8,expectedRead);
    }
}
