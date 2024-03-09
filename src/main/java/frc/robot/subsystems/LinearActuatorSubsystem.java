package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.SparkMax.Usage;
import static frc.robot.Constants.LinearActuator.*;

public class LinearActuatorSubsystem extends SubsystemBase {
    private final CANSparkMax actuatorMotor;
    private final RelativeEncoder encoder;
    private final SparkPIDController controller;

    public LinearActuatorSubsystem() {
        actuatorMotor = new CANSparkMax(MOTOR_CAN_ID, MotorType.kBrushed);

        encoder = actuatorMotor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 8192); // 2048 or 8192
        encoder.setPositionConversionFactor(1.0);
        encoder.setInverted(true);
        encoder.setPosition(0.0);

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

        Shuffleboard.getTab("debug").addNumber("LINEAR ACTUATOR HEIGHT", encoder::getPosition);
    }

    public void shift(boolean up, boolean down) {
        if (up && !down) {
            actuatorMotor.set(1.0);
        } else if (!up && down) {
            actuatorMotor.set(-1.0);
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
        if (p < ABSOLUTE_LOWEST || p > ABSOLUTE_HIGHEST) {
            controller.setReference(0, CANSparkBase.ControlType.kVelocity);
            actuatorMotor.set(0);
        }
    }
}
