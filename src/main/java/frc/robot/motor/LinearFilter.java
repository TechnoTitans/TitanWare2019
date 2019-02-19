package frc.robot.motor;

public class LinearFilter {
    private double maxRate;
    private double value;

    public LinearFilter(double maxRate) {
        this.maxRate = maxRate;
    }

    public void update(double newValue) {
        value = Math.min(value + maxRate, Math.max(newValue, value - maxRate));
    }

    public double getValue() {
        return value;
    }

    public void setValue(double value) {
        this.value = value;
    }
}
