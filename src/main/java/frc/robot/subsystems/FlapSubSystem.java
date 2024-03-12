package frc.robot.subsystems;

//import com.revrobotics.*;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Flap;
public class FlapSubSystem extends SubsystemBase {
    private final Servo flapServo = new Servo(Flap.FLAP_PWM_CHANNEL);
  boolean open;
  boolean amp;
  boolean source;
    
    public FlapSubSystem(){
           open = false;
           amp = false;
           source = false;
    }

    public void setFlapSource(boolean actuate){
        if (actuate&& open == false){
            flapServo.set(Flap.SOURCE_POSITION);
            open = true;
            source = true;
            amp = false;
        }
        else if (actuate && open == true && amp == true){
            flapServo.set(Flap.SOURCE_POSITION);
            source = true;
            amp = false;
        }
        else if (actuate && open == true && source == true){
            flapServo.set(Flap.CLOSED_POSITION);
            source = false;
            open = false;
            amp = false;
        }
        else{}

    }
        public void setFlapAmp(boolean actuate){
        if (actuate&& open == false){
            flapServo.set(Flap.AMP_POSITION);
            open = true;
            source = false;
            amp = true;
        }
        else if (actuate && open == true && source == true){
            flapServo.set(Flap.AMP_POSITION);
            source = false;
            amp = true;
        }
        else if (actuate && open == true && amp == true){
            flapServo.set(Flap.CLOSED_POSITION);
            source = false;
            open = false;
            amp = false;
        }
        else{}

    }


}
