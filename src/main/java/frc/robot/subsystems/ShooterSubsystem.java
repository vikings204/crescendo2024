package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    private boolean ignoreSensor = false;
    private final SendableChooser<Boolean> ignoreSensorChooser = new SendableChooser<>();
    private final LEDSubsystem led;
    private boolean noteDetected;

    public ShooterSubsystem(LEDSubsystem ledSubsystem) {
        led = ledSubsystem;

        shooterMotor_1 = new CANSparkMax(SHOOTER_MOTOR1_ID, MotorType.kBrushless);
        shooterMotor_2 = new CANSparkMax(SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        configMotors();

        sensor1 = new ColorSensorV3(I2C.Port.kOnboard);
        sensor1.configureProximitySensor(ColorSensorV3.ProximitySensorResolution.kProxRes11bit, ColorSensorV3.ProximitySensorMeasurementRate.kProxRate6ms);

        ignoreSensorChooser.addOption("deactivate sensor", false);
        ignoreSensorChooser.setDefaultOption("sensor is active", true);
        Shuffleboard.getTab("SmartDashboard").add("ignore intake sensor", ignoreSensorChooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withSize(2, 1); //SmartDashboard.putData("Ignore Intake Sensor", ignoreSensorChooser);
        ignoreSensorChooser.onChange((Boolean val) -> ignoreSensor = val);

        Shuffleboard.getTab("SmartDashboard").addNumber("Intake Motor Current", intakeMotor::getOutputCurrent); //System.out.println("Intake Motor Current "+intakeMotor.getOutputCurrent());
    }

    //configDriveMotor();
    private void configMotors() {
        shooterMotor_1.restoreFactoryDefaults();
        ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(shooterMotor_1, Usage.kAll);
        shooterMotor_1.setSmartCurrentLimit(CURRENT_LIMIT);
        shooterMotor_1.setInverted(SHOOTER_INVERT);
        shooterMotor_1.setIdleMode(IDLE_MODE);
        shooterMotor_1.enableVoltageCompensation(VOLTAGE_COMP);
        shooterMotor_1.burnFlash();

        shooterMotor_2.restoreFactoryDefaults();
        ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(shooterMotor_2, Usage.kAll);
        shooterMotor_2.setSmartCurrentLimit(CURRENT_LIMIT);
        shooterMotor_2.setInverted(!SHOOTER_INVERT);
        shooterMotor_2.setIdleMode(IDLE_MODE);
        shooterMotor_2.enableVoltageCompensation(VOLTAGE_COMP);
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
        if (shoot && !noteDetected) {
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
        if (ignoreSensor) {
            if (shoot) {
                intakeMotor.set(reverse ? -INTAKE_SPEED : INTAKE_SPEED);
            } else {
                intakeMotor.set(0);
            }
        } else {
            var detected = noteDetected;
            if (flywheelState || reverse) {
                detected = false;
            }

            if (shoot && !detected) {
                intakeMotor.set(reverse ? -INTAKE_SPEED : INTAKE_SPEED);
                // moved to init Shuffleboard.getTab("SmartDashboard").addNumber("Intake Motor Current", intakeMotor::getOutputCurrent); //System.out.println("Intake Motor Current "+intakeMotor.getOutputCurrent());
            } else {
                intakeMotor.set(0);
            }
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("NOTE DISTANCE", sensor1.getProximity());
        noteDetected = sensor1.getProximity() > INTAKE_SENSOR_THRESHOLD;

        if (noteDetected && !flywheelState) {
            led.Presets.HasNote();
        } else if (flywheelState) {
            led.Presets.Shooting();
        } else {
            led.Presets.Default();
        }
    }
}
