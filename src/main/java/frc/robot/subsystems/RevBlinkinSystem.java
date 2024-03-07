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
    
    public void AutonomousModeLight() {
        blinkin.set(0.65); //sets color to Orange when the robot is autonomous
    } 

    public void TeleopLight() {
        blinkin.set(0.75); //sets color to Dark Green when the robot is in teleop mode
    } 

    public void SuccessfulIntakeLight() {
        blinkin.set(0.71); //sets color to Lawn Green if the robot successfully intakes the ring
    }

    public void HoldingNoteLight(){
        blinkin.set(0.63); //sets color to Red Orance if the robot is currently in possession of the orange note.
    }

    public void PreLight() {
        blinkin.set(0.67); /*sets color to Gold if robot is about to do the following:
        about to shoot into the Speaker
        about to shoot into Amp
        about to attach to Hook
        */
    }

    public void SuccessfulPointLight() {
        blinkin.set(0.73); /*sets color to Lime if robot:
            successfully shoots the note into the speaker
            successfully makes it into the Amp
        */
    }

    public void HookLight() {
        blinkin.set(0.79); //sets color to Blue Green if the robot successfully attaches to chain with Hook
    }

    public void FailLight(){
        blinkin.set(0.95); //sets color to Gray if the robot fails to intake, misses Speaker shot, misses Amp shot, basically fails at task
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
