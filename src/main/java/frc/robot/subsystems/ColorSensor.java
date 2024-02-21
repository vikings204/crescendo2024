package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;

public class ColorSensor extends SubsystemBase {

    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);


    public void fuckinguselesscantdoshit() {

            //Proximity
            System.out.println("Proximity" + m_colorSensor.getProximity());

            int Proximity = m_colorSensor.getProximity();

            if (Proximity > 1800) {
                System.out.println("Practically touching the sensor");
            } else if (Proximity > 1000) {
                System.out.println("about 1.5cm or closer if not black");
            } else if (Proximity > 600) {
                System.out.println("about 2.25cm or closer if not black");
            } else if (Proximity> 350) {
                System.out.println("about 3cm or closer if not black");
            } else if (Proximity > 290) {
                System.out.println("about 4cm or closer if not black");
            } else if (Proximity > 215) {
                System.out.println("about 5cm or closer if not black");
            } else {
                System.out.println("greater than 5cm, cant relaly tell");
            }

    }
}