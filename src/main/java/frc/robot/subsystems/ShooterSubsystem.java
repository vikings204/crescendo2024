package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.SparkMax.Usage;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor_1;
    private final CANSparkMax shooterMotor_2;
    private final CANSparkMax bumpMotor;

    public ShooterSubsystem() {
        shooterMotor_1 = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR1_ID, MotorType.kBrushless);
        shooterMotor_2 = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
        bumpMotor = new CANSparkMax(Constants.Shooter.BUMP_MOTOR_ID, MotorType.kBrushless);
        configMotors();
    }

    //configDriveMotor();
    private void configMotors() {
        shooterMotor_1.restoreFactoryDefaults();
        ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(shooterMotor_1, Usage.kAll);
        shooterMotor_1.setSmartCurrentLimit(Constants.Shooter.CURRENT_LIMIT);
        shooterMotor_1.setInverted(Constants.Shooter.SHOOTER_INVERT);
        shooterMotor_1.setIdleMode(Constants.Shooter.IDLE_MODE);
        shooterMotor_1.enableVoltageCompensation(Constants.Shooter.voltageComp);
        shooterMotor_1.burnFlash();

        shooterMotor_2.restoreFactoryDefaults();
        ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(shooterMotor_2, Usage.kAll);
        shooterMotor_2.setSmartCurrentLimit(Constants.Shooter.CURRENT_LIMIT);
        shooterMotor_2.setInverted(!Constants.Shooter.SHOOTER_INVERT);
        shooterMotor_2.setIdleMode(Constants.Shooter.IDLE_MODE);
        shooterMotor_2.enableVoltageCompensation(Constants.Shooter.voltageComp);
        shooterMotor_2.burnFlash();

        bumpMotor.restoreFactoryDefaults();
        ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(bumpMotor, Usage.kAll);
        bumpMotor.setSmartCurrentLimit(Constants.Shooter.CURRENT_LIMIT);
        bumpMotor.setInverted(!Constants.Shooter.BUMP_INVERT);
        bumpMotor.setIdleMode(Constants.Shooter.IDLE_MODE);
        bumpMotor.burnFlash();
    }

    public void speakerShot(boolean shoot) {
        if (shoot) {
            shooterMotor_1.set(1);
            shooterMotor_2.set(1);
        } else {
            shooterMotor_1.set(0);
            shooterMotor_2.set(0);
            bumpMotor.set(0);
        }

    }

    public void ampShot(boolean shoot) {
        if (shoot) {
            shooterMotor_1.set(Constants.Shooter.AMP_STRENGTH);
            shooterMotor_2.set(Constants.Shooter.AMP_STRENGTH - .025);
        } else {
            shooterMotor_1.set(0);
            shooterMotor_2.set(0);
            bumpMotor.set(0);
        }
    }

    public void receive(boolean shoot) {
        if (shoot) {
            shooterMotor_1.set(-.05);
            shooterMotor_2.set(-.05);
            bumpMotor.set(0.8);
        } else {
            shooterMotor_1.set(0);
            shooterMotor_2.set(0);
            bumpMotor.set(0);
        }
    }

    public void bump(boolean shoot) {
        if (shoot) {
            bumpMotor.set(-0.5);
        } else {
            bumpMotor.set(0);
        }
    }
}
