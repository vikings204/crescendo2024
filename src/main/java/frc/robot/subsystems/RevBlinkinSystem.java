package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.PWM;

public class RevBlinkinSystem extends SubsystemBase{
    Spark blinkin;
    public RevBlinkinSystem() {
        blinkin = new Spark(9);
    }   
    
    //based on scoring
    public void setColorBasedOnPointSuccess(boolean SuccessfulPointLight) {
        if (SuccessfulPointLight) {
            blinkin.set(0.73); /*sets color to Lime if robot:
            successfully shoots the note into the speaker
            successfully makes it into the Amp
        */
        }

        else {
            blinkin.set(0.61); //sets color to Red if the robot fails to intake, misses Speaker shot, misses Amp shot, basically fails at task
        }
    }

    //based on mode
    public void setColorBasedOnMode(boolean AutoMode) {
        if (AutoMode) { //if in autonomous mode
            blinkin.set(0.63); //sets color to Red Orange when the robot is autonomous
        }

        else { //Teleop mode
            blinkin.set(0.75); //sets color to Dark Green when the robot is in teleop mode
           
        }
    }

    //based on note possession
    public void setColorBasedOnNote(boolean HoldingNote) {
        if (HoldingNote) { //if holding note
            blinkin.set(0.65); //sets color to Orange if the robot is currently in possession of the orange note.
        }

        else { //not holding note
            blinkin.set(0.69); //sets color to Yellow if the robot is currently in possession of the orange note.
           
        }
    }
    
    public void setColorBasedOnHookSuccess(boolean SuccessfulHookLight) {
        if (SuccessfulHookLight) { //successfully holds onto hook
            blinkin.set(0.79); //sets color to Blue Green if the robot successfully attaches to chain with Hook
        }

        else {
            blinkin.set(0.61); //sets color to Red if the robot fails to attach to Hook
        }
    }

    //based on Intake success
    public void setColorBasedOnIntakeSuccess(boolean SuccessfulIntakeLight) {
        if (SuccessfulIntakeLight) { //successfully intakes note
            blinkin.set(0.71); //sets color to Lawn Green if the robot successfully intakes the ring
        }

        else {
            blinkin.set(0.61); //sets color to Red if the robot fails to intake note
        }
    }

    //based on whether robot is preparing or not, no else condition because when it isnt preparing it's just doing basically any other condition
    public void setColorBasedOnPreparation(boolean PreLight) {
        if (PreLight) { //about to prepare to make a move
            blinkin.set(0.67); /*sets color to Gold if robot is about to do the following:
        about to shoot into the Speaker, the Amp OR the robot is about to attach to the hook
        */
        }
    }


    public class Spark extends PWMMotorController {
        
        @SuppressWarnings("this-escape")
        public Spark(final int channel) {
            super("Spark", 2);
        
            m_pwm.setBoundsMicroseconds(2003, 1550, 1500, 1460, 999);
            m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
            m_pwm.setSpeed(0.0);
            m_pwm.setZeroLatch();

            HAL.report(tResourceType.kResourceType_RevSPARK, getChannel() + 1);
        }
    }
}
