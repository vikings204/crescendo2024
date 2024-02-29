package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class RevBlinkinSystem extends SubsystemBase{
    Spark led;
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
    public class Robot extends TimedRobot {
        private DifferentialDrive m_robotDrive;
        private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
        private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);

        @Override
        public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
            m_rightMotor.setInverted(true);

            m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

            led = new Spark(9);
        }

        //MODES
        public void teleopInit() {
            led = new Spark(9);
            led.set(0.73);
        }

    }
}
