package frc.robot.util;

public class PolarCoordinate {
    public double mag;
    public double deg;

    public PolarCoordinate(double r, double theta) {
        this.mag = r;
        this.deg = theta;
    }
}
