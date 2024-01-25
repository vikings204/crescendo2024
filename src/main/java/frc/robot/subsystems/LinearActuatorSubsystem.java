package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class LinearActuatorSubsystem extends SubsystemBase {
    private final CANSparkMax actuatorMotor;

    public LinearActuatorSubsystem() {
        actuatorMotor = new CANSparkMax(27, MotorType.kBrushed);

        actuatorMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(actuatorMotor, Usage.kAll);
        actuatorMotor.setSmartCurrentLimit(Constants.Shooter.driveContinuousCurrentLimit);
        actuatorMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        actuatorMotor.enableVoltageCompensation(Constants.Shooter.voltageComp);
        actuatorMotor.burnFlash();
    }

    public void shift(boolean up, boolean down) {
        if (up && !down) {
            actuatorMotor.set(-1.0);
        } else if (!up && down) {
            actuatorMotor.set(1.0);
        } else {
            actuatorMotor.set(0);
        }
    }
}
