package frc.robot.subsystems;
import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FBLSubsystem extends SubsystemBase {
    private final CANSparkMax motor1; // left
    private final CANSparkMax motor2; // right
    private final RelativeEncoder motor1Encoder;
    private final RelativeEncoder motor2Encoder;
    private final SparkPIDController motor1Controller;
    private final SparkPIDController motor2Controller;

    public FBLSubsystem() {
        motor1 = new CANSparkMax(Constants.FourBarLinkage.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        motor1Encoder = motor1.getEncoder();
        motor1Controller = motor1.getPIDController();

        motor2 = new CANSparkMax(Constants.FourBarLinkage.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
        motor2Encoder = motor2.getEncoder();
        motor2Controller = motor2.getPIDController();

        configure();
    }

    private void configure() {
        motor1.restoreFactoryDefaults();
        motor1.setSmartCurrentLimit(Constants.FourBarLinkage.CURRENT_LIMIT);
        motor1.setInverted(Constants.FourBarLinkage.INVERT);
        motor1.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor1Controller.setP(Constants.FourBarLinkage.PID_P);
        motor1Controller.setI(Constants.FourBarLinkage.PID_I);
        motor1Controller.setD(Constants.FourBarLinkage.PID_D);
        motor1Controller.setFF(Constants.FourBarLinkage.PID_FF);
        motor1.burnFlash();
        motor1Encoder.setPositionConversionFactor(Constants.FourBarLinkage.ANGLE_CONVERSION);
        motor1Encoder.setPosition(0.0);

        motor2.restoreFactoryDefaults();
        motor2.setSmartCurrentLimit(Constants.FourBarLinkage.CURRENT_LIMIT);
        motor2.setInverted(!Constants.FourBarLinkage.INVERT);
        motor2.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor2Controller.setP(Constants.FourBarLinkage.PID_P);
        motor2Controller.setI(Constants.FourBarLinkage.PID_I);
        motor2Controller.setD(Constants.FourBarLinkage.PID_D);
        motor2Controller.setFF(Constants.FourBarLinkage.PID_FF);
        motor2.burnFlash();
        motor2Encoder.setPositionConversionFactor(Constants.FourBarLinkage.ANGLE_CONVERSION);
        motor2Encoder.setPosition(0.0);
    }

    public void goLowest() {
        motor1Controller.setReference(Constants.FourBarLinkage.MAXIMUM_ANGLE, ControlType.kPosition);
        motor2Controller.setReference(Constants.FourBarLinkage.MAXIMUM_ANGLE, ControlType.kPosition);
    }
    public void goHighest() {
        motor1Controller.setReference(Constants.FourBarLinkage.MINIMUM_ANGLE, ControlType.kPosition);
        motor2Controller.setReference(Constants.FourBarLinkage.MINIMUM_ANGLE, ControlType.kPosition);
    }

    public void goPosition(double degrees) {
        if (degrees > Constants.FourBarLinkage.MAXIMUM_ANGLE || degrees < Constants.FourBarLinkage.MINIMUM_ANGLE) {
            return;
        }

        motor1Controller.setReference(degrees, ControlType.kPosition);
        motor2Controller.setReference(degrees, ControlType.kPosition);
    }
}
