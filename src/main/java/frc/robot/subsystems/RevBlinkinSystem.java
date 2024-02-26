package frc.robot.subsystems;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class RevBlinkinSystem {
    public class RevBlinkin extends SubsystemBase {
        public RevBlinkin() {
            Spark revblinkin = Spark(2);
    }

    //MODES
    //If the robot is in test mode the LED turns solid white(0.93)
    public void lightsTest() {
        revblinkin.set(0.93);
    }

    //If the robot is in auto mode the LED turns solid red orange(0.63)
    public void lightsAuto() {
        revblinkin.set(0.63);
    }

    //If the robot is in teleop mode the LED turns solid sky blue(0.83)
    public void lightsTeleop() {
        revblinkin.set(0.83);
    }
    //If robot detects shooter, turns green(0.77)
    public void ShooterLight() {
        revblinkin.set(0.77);
    }

    //robot about to use hook, turn orange(0.65)
    public void HookLight() {
        revblinkin.set(0.65);
    }

    //robot intaking, turn aqua(0.81)
    public void IntakeLight(){
        revblinkin.set(0.81);
    }

    //robot intaking, turn aqua(0.81)
    public void IntakeLight(){
        revblinkin.set(0.81);
    }
}

}
