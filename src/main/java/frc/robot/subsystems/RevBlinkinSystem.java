package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.PWM;

public class RevBlinkinSystem extends SubsystemBase{
    Spark blinkin;
    public RevBlinkinSystem() {
        blinkin = new Spark(9);
    }   
    
    public void SuccessfulIntakeLight() {
        blinkin.set(0.71); //sets color to Lawn Green if the robot successfully intakes the ring
    }

    public void AboutToShootLight() {
        blinkin.set(0.65); //sets color to Orange when the robot is about to shoot
    }    

    public void SuccessfulShot() {
        blinkin.set(0.73); //sets color to Lime when the robot makes it into the Speaker
    } 

    public void AutonomousModeLight() {
        blinkin.set(0.63); //sets color to Red Orange when the robot is autonomous
    } 

    public void StartHookLight() {
        blinkin.set(0.89); //sets color to Blue Violet when the robot's about to latch its hook on the chain
    } 

    public void HookLight() {
        blinkin.set(0.91); //sets color to  Violet when the robot latches its hook on the chain
    } 

    public void TeleopLight() {
        blinkin.set(0.75); //sets color to Dark Green when the robot is in teleop mode
    } 

    public void PreAmpLight() {
        blinkin.set(0.59); //sets color to Dark Red when the robot is about to shoot the note into the amp
    } 
    
    public void AmpLight() {
        blinkin.set(0.61); //sets color to Red when the robot is about to shoot the note into the amp
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
