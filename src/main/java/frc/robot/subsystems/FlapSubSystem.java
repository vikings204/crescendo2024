package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Flap;

public class FlapSubSystem extends SubsystemBase {
    private final Servo flapServo = new Servo(Flap.FLAP_PWM_CHANNEL);
    boolean open;
    boolean amp;
    boolean source;
    private double setpoint = 0.2;

    public FlapSubSystem() {
        open = false;
        amp = false;
        source = false;

        Shuffleboard.getTab("main").addNumber("servo linear actuator", () -> setpoint);
    }

    public void setFlapSource(boolean actuate) {
        if (actuate && open == false) {
            flapServo.set(Flap.SOURCE_POSITION);
            open = true;
            source = true;
            amp = false;
            System.out.println("Go to Source");
        } else if (actuate && open == true && amp == true) {
            flapServo.set(Flap.SOURCE_POSITION);
            source = true;
            amp = false;
            System.out.println("Go to Source");
        } else if (actuate && open == true && source == true) {
            flapServo.set(Flap.CLOSED_POSITION);
            source = false;
            open = false;
            amp = false;
            System.out.println("Go to Close");
        } else {
        }

    }

    public void setFlapAmp(boolean actuate) {
        if (actuate && open == false) {
            flapServo.set(Flap.AMP_POSITION);
            open = true;
            source = false;
            amp = true;
        } else if (actuate && open == true && source == true) {
            flapServo.set(Flap.AMP_POSITION);
            source = false;
            amp = true;
        } else if (actuate && open == true && amp == true) {
            flapServo.set(Flap.CLOSED_POSITION);
            source = false;
            open = false;
            amp = false;
        } else {
        }

    }

    public void shift(boolean up, boolean down) {
        if (up) {
            setpoint = 0.2;
        } else if (down) {
            setpoint = 0.5;
        }
        flapServo.set(setpoint);
    }
}
