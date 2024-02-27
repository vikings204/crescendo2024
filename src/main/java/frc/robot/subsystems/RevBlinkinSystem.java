package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.PWM;

public class RevBlinkinSystem extends SubsystemBase{
    public class Spark extends PWMMotorController {
        @SuppressWarnings("this-escape")
        protected Spark(String name, int channel) {
            super("Spark", 2);
            //TODO Auto-generated constructor stub
            m_pwm.setBoundsMicroseconds(2003, 1550, 1500, 1460, 999);
            m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
            m_pwm.setSpeed(0.0);
            m_pwm.setZeroLatch();
        
            HAL.report(tResourceType.kResourceType_RevSPARK, getChannel() + 1);
        }
        
    }
}
