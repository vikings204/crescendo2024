package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants204.Arm;
import frc.robot.Constants204.ArmCAN;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax boomMotor = new CANSparkMax(ArmCAN.BOOM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final SparkMaxLimitSwitch boomForwardLimit = boomMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    public final RelativeEncoder boomEncoder = boomMotor.getEncoder();
    private final SparkMaxPIDController boomPIDCon = boomMotor.getPIDController();
    private final CANSparkMax dipperMotor = new CANSparkMax(ArmCAN.DIPPER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public final RelativeEncoder dipperEncoder = dipperMotor.getEncoder();
    private final SparkMaxPIDController dipperPIDCon = dipperMotor.getPIDController();
    private final Servo clawServo = new Servo(ArmCAN.CLAW_SERVO_PWM_CH);
    private boolean clawState = false;
    public double boomStart = 0.0;
    public double boomMax = 25.0;
    public double dipperMax = 0.0;
    public double dipperAuto = 50.0;
    private double clawSetPoint = 1.0;

    private final double kP = 0.5;
    private final double kI = 1e-4;
    private final double kD = 1;
    private final double kIz = 0;
    private final double kFF = 0;
    private final double kMaxOutput = 1;
    private final double kMinOutput = -1;

    public ArmSubsystem() {
        boomPIDCon.setP(kP);
        boomPIDCon.setI(kI);
        boomPIDCon.setD(kD);
        boomPIDCon.setIZone(kIz);
        boomPIDCon.setFF(kFF);
        //boomPIDCon.setOutputRange(kMinOutput, kMaxOutput);
        boomMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        //boomForwardLimit.enableLimitSwitch(true);
        boomEncoder.setPosition(0);

        dipperPIDCon.setP(kP);
        dipperPIDCon.setI(kI);
        dipperPIDCon.setD(kD);
        dipperPIDCon.setIZone(kIz);
        dipperPIDCon.setFF(kFF);
        //dipperPIDCon.setOutputRange(kMinOutput, kMaxOutput);
        dipperMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        dipperEncoder.setPosition(0);
    }

    /**
     * boom increment, dipper increment, claw binary<br>
     * increment: 0=none, >0=increase, <0=decrease<br>
     * binary: 0=none, >0=opened, <0=closed<br>
     **/
    public void setArm(double b, double d, double c) {
        //System.out.println("B:"+b + "D:"+d + "C:"+c);
        double nb = boomEncoder.getPosition();
       // System.out.println("Boom Enconder Value: " + nb);
        double nd = dipperEncoder.getPosition();
        if (b == 0) {
        } else if (b > 0 && nb < (boomStart + boomMax)) {
            nb += Arm.BOOM_REF_INCREMENT;
        } else if (b < 0) {
            nb -= Arm.BOOM_REF_INCREMENT;
        }/*
        } else if (b > 0) {
            nb += Arm.BOOM_REF_INCREMENT;
        } else if (b < 0) {
            nb -= Arm.BOOM_REF_INCREMENT;
        }*/
        if (d == 0) {
        } else if (d > 0 && nd < dipperMax) {
            nd += Arm.DIPPER_REF_INCREMENT;
        } else if (d < 0) {
            nd -= Arm.DIPPER_REF_INCREMENT;
        }

        boomPIDCon.setReference(nb, CANSparkMax.ControlType.kPosition);
        dipperPIDCon.setReference(nd, CANSparkMax.ControlType.kPosition);
        if (c == 0) {
            if (clawState) {
                clawServo.set(clawSetPoint);
            } else {
                clawServo.set(clawSetPoint);
            }
        } else if (c < 0 && clawSetPoint > 0) {
            clawState = true;
            clawSetPoint -= .05;
            clawServo.set(clawSetPoint);
        } else if (c > 0 && clawSetPoint < 1.0) {
            clawState = true;
            clawSetPoint += .05;
            clawServo.set(clawSetPoint);
        }
        //System.out.println("Claw SP: "+ clawSetPoint);
    }

    public void setArmTest(double b, double d, double c) {
        //System.out.println("B:"+b + "D:"+d + "C:"+c);
        double nb = boomEncoder.getPosition();
        double nd = dipperEncoder.getPosition();
        //System.out.println("Dipper Enconder Value: " + nd);
        //System.out.println("Forward Limit Enabled" + boomForwardLimit.isLimitSwitchEnabled() + " Limit Switch:" + boomForwardLimit.isPressed());
        if (b == 0) {
        } else if (b > 0) {
            nb += Arm.BOOM_REF_INCREMENT;

        } else if (b < 0) {
            nb -= Arm.BOOM_REF_INCREMENT;
        }
        if (d == 0) {
        } else if (d > 0) {
            nd += Arm.DIPPER_REF_INCREMENT;
        } else if (d < 0) {
            nd -= Arm.DIPPER_REF_INCREMENT;
        }
        //System.out.println("TESTBoom Enconder Value: "+nb);
        //Maybe add some code here to not move the arm unless something is pushed

        boomPIDCon.setReference(nb, CANSparkMax.ControlType.kPosition);
        dipperPIDCon.setReference(nd, CANSparkMax.ControlType.kPosition);
        if (c == 0) {
            if (clawState) {
                clawServo.set(clawSetPoint);
            } else {
                clawServo.set(clawSetPoint);
            }
        } else if (c < 0 && clawSetPoint > 0) {
            clawState = true;
            clawSetPoint -= .05;
            clawServo.set(clawSetPoint);
        } else if (c > 0 && clawSetPoint < 1.0) {
            clawState = true;
            clawSetPoint += .05;
            clawServo.set(clawSetPoint);
        }
        //System.out.println("Claw SP: "+ clawSetPoint);
    }

    /*private double setServoScaleFix(double joyIn) {
        return (joyIn+1)/2;
    }*/
}