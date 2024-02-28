package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.SparkMax.Usage;

import static frc.robot.Constants.Shooter.*;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor_1;
    private final CANSparkMax shooterMotor_2;
    private final CANSparkMax intakeMotor;
    private final ColorSensorV3 sensor1;
    private boolean flywheelState = false;

    public ShooterSubsystem() {
        shooterMotor_1 = new CANSparkMax(SHOOTER_MOTOR1_ID, MotorType.kBrushless);
        shooterMotor_2 = new CANSparkMax(SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        configMotors();

        sensor1 = new ColorSensorV3(I2C.Port.kOnboard);
        sensor1.configureProximitySensor(ColorSensorV3.ProximitySensorResolution.kProxRes11bit, ColorSensorV3.ProximitySensorMeasurementRate.kProxRate6ms);
    }

    //configDriveMotor();
    private void configMotors() {
        shooterMotor_1.restoreFactoryDefaults();
        ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(shooterMotor_1, Usage.kAll);
        shooterMotor_1.setSmartCurrentLimit(CURRENT_LIMIT);
        shooterMotor_1.setInverted(SHOOTER_INVERT);
        shooterMotor_1.setIdleMode(IDLE_MODE);
        shooterMotor_1.enableVoltageCompensation(voltageComp);
        shooterMotor_1.burnFlash();

        shooterMotor_2.restoreFactoryDefaults();
        ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(shooterMotor_2, Usage.kAll);
        shooterMotor_2.setSmartCurrentLimit(CURRENT_LIMIT);
        shooterMotor_2.setInverted(!SHOOTER_INVERT);
        shooterMotor_2.setIdleMode(IDLE_MODE);
        shooterMotor_2.enableVoltageCompensation(voltageComp);
        shooterMotor_2.burnFlash();

        intakeMotor.restoreFactoryDefaults();
        ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(intakeMotor, Usage.kAll);
        intakeMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        intakeMotor.setInverted(!BUMP_INVERT);
        intakeMotor.setIdleMode(IDLE_MODE);
        intakeMotor.burnFlash();
    }

    public void flywheelSpeaker(boolean shoot) {
        flywheelState = shoot;
        if (shoot) {
            shooterMotor_1.set(SPEAKER_SPEED);
            shooterMotor_2.set(SPEAKER_SPEED);
        } else {
            shooterMotor_1.set(0);
            shooterMotor_2.set(0);
            intakeMotor.set(0);
        }

    }

    public void flywheelAmp(boolean shoot) {
        flywheelState = shoot;
        if (shoot) {
            shooterMotor_1.set(AMP_SPEED);
//            shooterMotor_2.set(AMP_SPEED - .025);
            shooterMotor_2.set(AMP_SPEED);
        } else {
            shooterMotor_1.set(0);
            shooterMotor_2.set(0);
            intakeMotor.set(0);
        }
    }

    public void receive(boolean shoot) {
        SmartDashboard.putNumber("DISTANCE", sensor1.getProximity());

        if (shoot && sensor1.getProximity() < INTAKE_SENSOR_THRESHOLD) {
            shooterMotor_1.set(-.05);
            shooterMotor_2.set(-.05);
            intakeMotor.set(0.8);
        } else {
            shooterMotor_1.set(0);
            shooterMotor_2.set(0);
            intakeMotor.set(0);
        }
    }

    public void intake(boolean shoot, boolean reverse) {
        SmartDashboard.putNumber("DISTANCE", sensor1.getProximity());
        var detected = sensor1.getProximity() > INTAKE_SENSOR_THRESHOLD;

        if (flywheelState) {
            detected = false;
        }

        if (shoot && !detected) {
            intakeMotor.set(reverse ? -INTAKE_SPEED : INTAKE_SPEED);
        } else {
            intakeMotor.set(0);
        }
    }
}
