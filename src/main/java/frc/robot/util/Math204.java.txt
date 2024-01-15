package frc.robot.util;

public class Math204 {
    public static PolarCoordinate CartesianToPolar(double x, double y) {
        double r = Math.sqrt(x*x + y*y);
        double rad = Math.atan2(y, x); // returns the angle in radians

        // Convert radians to degrees
        //double deg = (Math.toDegrees(rad) + 90)*-1;
        double deg = Math.toDegrees(rad);

        // conv
        if (deg < 0) {
            deg = (deg+360);
        }

        return new PolarCoordinate(r, deg);
    }

    public static int GetQuadrant(double deg) {
        double q = deg;

        if (0 <= q && q < 90) {
            q = 1;
        } else if (90 <= q && q < 180) {
            q = 2;
        } else if (180 <= q && q < 270) {
            q = 3;
        } else if (270 <= q) {
            q = 4;
        }

        return (int) q;
    }
}
