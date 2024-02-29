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
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.SparkMax.Usage;

import static frc.robot.Constants.Swerve.*;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private double turningPDeg = 0;
    private int turningPQuad = 0; // top left is 1 counterclockwise
    private double turningTotalDeg = 0.0;

    //public final TalonSRX angleMotor;
    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder integratedAngleEncoder;
    //private CANcoder angleEncoder;

    private final SparkPIDController driveController;
    private final SparkPIDController angleController;
    //private final TalonSRXFeedbackDevice angleController;

    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    DRIVE_FF_S, DRIVE_FF_V, DRIVE_FF_A);

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, Rotation2d angleOffset) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        /* Angle Encoder Config */
        // angleEncoder = new CANCoder(moduleConstants.cancoderID);
        //configAngleEncoder();

        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();


        /* Drive Motor Config */
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
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

        //angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);
        angleMotor.setInverted(ANGLE_INVERT);
        angleMotor.setIdleMode(ANGLE_IDLE_MODE);

        integratedAngleEncoder.setPositionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);
        angleController.setP(ANGLE_PID_P);
        angleController.setI(ANGLE_PID_I);
        angleController.setD(ANGLE_PID_D);
        angleController.setFF(ANGLE_PID_FF);
        angleMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        angleMotor.burnFlash();
        Timer.delay(1);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);
        driveMotor.setInverted(DRIVE_INVERT);
        driveMotor.setIdleMode(DRIVE_IDLE_MODE);
        driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION_FACTOR);
        driveEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR);
        driveController.setP(DRIVE_PID_P);
        driveController.setI(DRIVE_PID_I);
        driveController.setD(DRIVE_PID_D);
        driveController.setFF(DRIVE_PID_FF);
        driveMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / MAX_SPEED;
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
                (Math.abs(desiredState.speedMetersPerSecond) <= (MAX_SPEED * 0.01))
                        ? lastAngle
                        : desiredState.angle;
            /*SmartDashboard.putNumber("Angle Position Setting Mod" + moduleNumber, angle.getDegrees());
            SmartDashboard.putNumber("Encoder Position Setting without Offset" + moduleNumber, (((angle.getDegrees())/360)*1023));   
            SmartDashboard.putNumber("Encoder Position Setting with Offset" + moduleNumber, (((angle.getDegrees()+angleOffset.getDegrees())/360)*1023)); */

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
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
}
