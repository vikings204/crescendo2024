package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.SparkMax.Usage;
import static frc.robot.Constants.LinearActuator.*;

public class LinearActuatorSubsystem extends SubsystemBase {
    private final CANSparkMax actuatorMotor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkPIDController controller;

    public LinearActuatorSubsystem() {
        actuatorMotor = new CANSparkMax(MOTOR_CAN_ID, MotorType.kBrushed);
        encoder = actuatorMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        encoder.setPositionConversionFactor(1.0); // need to configure with very low number to ensure pid isn't constantly trying to fix

        controller = actuatorMotor.getPIDController();
        controller.setFeedbackDevice(encoder);
        controller.setP(PID_P);
        controller.setI(PID_I);
        controller.setD(PID_D);
        controller.setFF(PID_FF);

        actuatorMotor.restoreFactoryDefaults();
        ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(actuatorMotor, Usage.kPositionOnly);
        actuatorMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        actuatorMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
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

    public void goToPosition(Position pos) {
        controller.setReference(pos.position, CANSparkBase.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // ensure it does not go outside its physical bounds (not sure if this works)
        var p = encoder.getPosition();
        if (p < ABSOLUTE_MINIMUM || p > ABSOLUTE_MAXIMUM) {
            controller.setReference(0, CANSparkBase.ControlType.kVelocity);
            actuatorMotor.set(0);
        }

        SmartDashboard.putNumber("LINEAR ACTUATOR HEIGHT", encoder.getPosition());
    }
}
