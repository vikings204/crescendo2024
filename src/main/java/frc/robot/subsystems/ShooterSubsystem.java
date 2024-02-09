package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotor_1;
    private final CANSparkMax shooterMotor_2;
    private final CANSparkMax bumpMotor;

    private final RelativeEncoder shooterRelativeEncoder_1;
    private final RelativeEncoder shooterRelativeEncoder_2;


    private final SparkPIDController shooterPidController_1;
    private final SparkPIDController shooterPidController_2;

    /*private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Shooter.shooterKS, Constants.Shooter.shooterKV, Constants.Shooter.shooterKA);*/
    public ShooterSubsystem() {
        shooterMotor_1 = new CANSparkMax(Constants.Shooter.shooterID_1, MotorType.kBrushless);
        shooterRelativeEncoder_1 = shooterMotor_1.getEncoder();
        shooterPidController_1 = shooterMotor_1.getPIDController();

        shooterMotor_2 = new CANSparkMax(Constants.Shooter.shooterID_2, MotorType.kBrushless);
        shooterRelativeEncoder_2 = shooterMotor_2.getEncoder();
        shooterPidController_2 = shooterMotor_2.getPIDController();

        bumpMotor = new CANSparkMax(Constants.Shooter.bumpID, MotorType.kBrushless);
        configShooterMotors();
    }

    //configDriveMotor();
    private void configShooterMotors() {
        shooterMotor_1.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(shooterMotor_1, Usage.kAll);
        shooterMotor_1.setSmartCurrentLimit(Constants.Shooter.driveContinuousCurrentLimit);
        shooterMotor_1.setInverted(Constants.Shooter.driveInvert);
        shooterMotor_1.setIdleMode(Constants.Shooter.driveNeutralMode);
        // shooterRelativeEncoder_1.setVelocityConversionFactor(Constants.Shooter.driveConversionVelocityFactor);
        // shooterRelativeEncoder_1.setPositionConversionFactor(Constants.Shooter.driveConversionPositionFactor);
        shooterPidController_1.setP(Constants.Shooter.shooterKP);
        shooterPidController_1.setI(Constants.Shooter.shooterKI);
        shooterPidController_1.setD(Constants.Shooter.shooterKD);
        shooterPidController_1.setFF(Constants.Shooter.shooterKFF);
        shooterMotor_1.enableVoltageCompensation(Constants.Shooter.voltageComp);
        shooterMotor_1.burnFlash();
        shooterRelativeEncoder_1.setPosition(0.0);

        shooterMotor_2.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(shooterMotor_2, Usage.kAll);
        shooterMotor_2.setSmartCurrentLimit(Constants.Shooter.driveContinuousCurrentLimit);
        shooterMotor_2.setInverted(!Constants.Shooter.driveInvert);
        shooterMotor_2.setIdleMode(Constants.Shooter.driveNeutralMode);
        // shooterRelativeEncoder_1.setVelocityConversionFactor(Constants.Shooter.driveConversionVelocityFactor);
        // shooterRelativeEncoder_1.setPositionConversionFactor(Constants.Shooter.driveConversionPositionFactor);
       // shooterPidController_2.setP(Constants.Shooter.shooterKP);
        //shooterPidController_2.setI(Constants.Shooter.shooterKI);
        //shooterPidController_2.setD(Constants.Shooter.shooterKD);
        //shooterPidController_2.setFF(Constants.Shooter.shooterKFF);
        //shooterMotor_2.enableVoltageCompensation(Constants.Shooter.voltageComp);
        shooterMotor_2.burnFlash();
        shooterRelativeEncoder_2.setPosition(0.0);

        bumpMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(bumpMotor, Usage.kAll);
        bumpMotor.setSmartCurrentLimit(Constants.Shooter.driveContinuousCurrentLimit);
        bumpMotor.setInverted(!Constants.Shooter.driveInvert);
        bumpMotor.setIdleMode(Constants.Shooter.driveNeutralMode);
        bumpMotor.burnFlash();
    }

    public void speakerShot(boolean shoot) {
        if (shoot) {
            
           /*  shooterPidController_1.setReference(
                    Constants.Shooter.speakerStrength,
                    ControlType.kVelocity);
            shooterPidController_2.setReference(
                    Constants.Shooter.speakerStrength,
                    ControlType.kVelocity);*/
                    shooterMotor_1.set(1.0);
                    shooterMotor_2.set(1.0);
                               /*  shooterPidController_1.setReference(
                    Constants.Shooter.speakerStrength,
                    ControlType.kVelocity);
            shooterPidController_2.setReference(
                    Constants.Shooter.speakerStrength,
                    ControlType.kVelocity);*/
                    //Rotation2d r1 = Rotation2d.fromDegrees(720);
                    //shooterPidController_1.setReference(720.0, ControlType.kPosition);
                    //shooterMotor_2.set(1.0);
        } else {
            shooterPidController_1.setReference(
                    0,
                    ControlType.kVelocity);
            shooterMotor_1.set(0);
            shooterPidController_2.setReference(
                    0,
                    ControlType.kVelocity);
            shooterMotor_2.set(0);
            bumpMotor.set(0);
        }

    }
        public void speakerShot(boolean shoot, int i) {
        if (shoot) {
            
   
                    //Rotation2d r1 = Rotation2d.fromDegrees(720);
                    shooterPidController_1.setReference(-1020.0, ControlType.kPosition);
                    //shooterMotor_2.set(1.0);
        } else {
            shooterPidController_1.setReference(
                    0,
                    ControlType.kVelocity);
            shooterMotor_1.set(0);
            shooterPidController_2.setReference(
                    0,
                    ControlType.kVelocity);
            shooterMotor_2.set(0);
            bumpMotor.set(0);
        }

    }

    

    public void ampShot(boolean shoot) {
        if (shoot) {
                   shooterMotor_1.set(Constants.Shooter.ampStrength);
                    shooterMotor_2.set(Constants.Shooter.ampStrength-.025);
        } else {
            shooterPidController_1.setReference(
                    0,
                    ControlType.kVelocity);
            shooterMotor_1.set(0);
            shooterPidController_2.setReference(
                    0,
                    ControlType.kVelocity);
            shooterMotor_2.set(0);
            bumpMotor.set(0);
        }
    }
        public void receive(boolean shoot) {
        if (shoot) {
            shooterMotor_1.set(-.05);
            shooterMotor_2.set(-.05);
            bumpMotor.set(-.05);
        } else {
            shooterMotor_1.set(0);
            shooterMotor_2.set(0);
            bumpMotor.set(0);
        }
    }
            public void bump(boolean shoot) {
        if (shoot) {
            bumpMotor.set(1);
            //shooterMotor_2.set(-.15);
        } 
        else {
          //  shooterPidController_1.setReference(
            //        0,
              //      ControlType.kVelocity);
            //shooterMotor_1.set(0);
            //shooterPidController_2.setReference(
              //      0,
                //    ControlType.kVelocity);
            //shooterMotor_2.set(0);
        }
    }
}
