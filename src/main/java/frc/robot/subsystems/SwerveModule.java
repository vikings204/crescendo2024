package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private double turningPDeg = 0;
    private int turningPQuad = 0; // top left is 1 counterclockwise
    private double turningTotalDeg = 0.0;

    //public final TalonSRX angleMotor;
    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;

    private final SparkPIDController driveController;
    private final SparkPIDController angleController;
    //private final TalonSRXFeedbackDevice angleController;

    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        // angleEncoder = new CANCoder(moduleConstants.cancoderID);
        //configAngleEncoder();

        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();


        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // REV and CTRE are not

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void resetToAbsolute() {
        //double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();

        integratedAngleEncoder.setPosition(0.0);
        System.out.println("Current Encoder Postition for Module " + moduleNumber + " is: " + getAngle());
    }

    private void configAngleEncoder() {
        //angleEncoder.configFactoryDefault();
        // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);

        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        angleMotor.setInverted(Constants.Swerve.angleInvert);
        angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);

        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleController.setFF(Constants.Swerve.angleKFF);
        angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        angleMotor.burnFlash();
        Timer.delay(1);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.Swerve.driveInvert);
        driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        driveController.setP(Constants.Swerve.driveKP);
        driveController.setI(Constants.Swerve.driveKI);
        driveController.setD(Constants.Swerve.driveKD);
        driveController.setFF(Constants.Swerve.driveKFF);
        driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                        ? lastAngle
                        : desiredState.angle;
            /*SmartDashboard.putNumber("Angle Position Setting Mod" + moduleNumber, angle.getDegrees());
            SmartDashboard.putNumber("Encoder Position Setting without Offset" + moduleNumber, (((angle.getDegrees())/360)*1023));   
            SmartDashboard.putNumber("Encoder Position Setting with Offset" + moduleNumber, (((angle.getDegrees()+angleOffset.getDegrees())/360)*1023)); */
        //angleMotor.set(TalonSRXControlMode.Position, (((angle.getDegrees()+angleOffset.getDegrees())/360)*1023));
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);

        double setter = SwerveContinuous(angle.getDegrees());
        //angleMotor.set(TalonSRXControlMode.Position, (((setter+angleOffset.getDegrees())/360)*1023));
        lastAngle = angle;
    }

    public void setAngleForX(double angle) {
        driveMotor.set(0);
        //angleMotor.set(TalonSRXControlMode.Position, (angle/360)*1023);
        angleController.setReference(angle, ControlType.kPosition);
    }

    private Rotation2d getAngle() {
        //SmartDashboard.putNumber("getAngleCall position Mod" + moduleNumber, (angleMotor.getSelectedSensorPosition()/1023)*360-angleOffset.getDegrees());
        //System.out.println("Encoder Position Mod "+moduleNumber+": "+(angleMotor.getSelectedSensorPosition()/1023)*360);
        //return Rotation2d.fromDegrees(((angleMotor.getSelectedSensorPosition()/1023)*360)-angleOffset.getDegrees());
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder() {
        // return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
        //return Rotation2d.fromDegrees(((angleMotor.getSelectedSensorPosition()/1023)*360)-angleOffset.getDegrees());
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public void resetEncoder() {
        integratedAngleEncoder.setPosition(0);
    }

    public SwerveModulePosition getPosition() {
        //SmartDashboard.putNumber("Raw Angle Reading " + moduleNumber, (angleMotor.getSelectedSensorPosition()/1023)*360);
        SmartDashboard.putNumber("angleEncoderCurrent Reading " + moduleNumber, integratedAngleEncoder.getPosition());
        //System.out.println("Encoder Position: "+((angleMotor.getSelectedSensorPosition()/1023)*360-angleOffset.getDegrees()));
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                //Rotation2d.fromDegrees((angleMotor.getSelectedSensorPosition()/1023)*360- angleOffset.getDegrees())
                Rotation2d.fromDegrees(integratedAngleEncoder.getPosition())
        );

    }

    private double SwerveContinuous(double cDeg) {
        double nDeg;
        int cQuad;

        if (0 <= cDeg && cDeg < 90) {
            cQuad = 1;
        } else if (90 <= cDeg && cDeg < 180) {
            cQuad = 2;
        } else if (180 <= cDeg && cDeg < 270) {
            cQuad = 3;
        } else {
            cQuad = 4;
        }

        if ((turningPQuad == 3 || turningPQuad == 4) && cQuad == 1) {
            nDeg = (360 - turningPDeg) + cDeg;
        } else if ((turningPQuad == 1 || turningPQuad == 2) && cQuad == 4) {
            nDeg = -((360 - cDeg) + turningPDeg);
        } else {
            nDeg = cDeg - turningPDeg;
        }

        turningTotalDeg += nDeg;
        turningPDeg = cDeg;
        turningPQuad = cQuad;
        return turningTotalDeg;
    }
}
