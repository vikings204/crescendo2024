package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class revcolormodule {
    Spark blinkin;
    public void setColor(double color) {
        blinkin.set(color);
    }
}