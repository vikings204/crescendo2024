// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants204;
import frc.robot.Constants.ModuleConstants;

import static frc.robot.Constants204.Controller.LX_MAG_DEADBAND;
import static frc.robot.Constants204.Drivetrain.*;
import frc.robot.util.PolarCoordinate;


public class StrafeModule {
    private final CANSparkMax driveMotor;
    public final TalonSRX turningMotor;
    private final RelativeEncoder driveEncoder;

    // private final PIDController drivePIDCon = new PIDController(0.1, 1e-4, 1);
    private SparkMaxPIDController drivePIDCon;

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController turningPIDCon =
            new ProfiledPIDController(
                    ModuleConstants.kPModuleTurningController,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(
                            ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                            ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    public StrafeModule(int driveMotorChannel,
                        int turningMotorChannel) {
        driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        turningMotor = new TalonSRX(turningMotorChannel);

        drivePIDCon = driveMotor.getPIDController();
        drivePIDCon.setP(STRAFE_DRIVE_PID_P);
        drivePIDCon.setI(1e-4);
        drivePIDCon.setD(1);
        drivePIDCon.setIZone(0);
        drivePIDCon.setFF(0);
        drivePIDCon.setOutputRange(-1, 1);

        turningMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0); //Set the feedback device that is hooked up to the talon

        turningMotor.setSelectedSensorPosition(0);

        turningMotor.config_kP(0, STRAFE_TURNING_PID_P);
        turningMotor.config_kI(0, STRAFE_TURNING_PID_I);
        turningMotor.config_kD(0, STRAFE_TURNING_PID_D);

        turningMotor.setNeutralMode(NeutralMode.Brake);
        ////////////////////////////
        //                        // 
        // MAYBECOMMENTMEOUT!!!!!!//
        //                        //
        ////////////////////////////

        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();

        turningMotor.configSelectedFeedbackCoefficient(1);

        // Set whether turning encoder should be reversed or not
        //m_turningMotor.setInverted(Constants.DriveConstants.kFrontLeftTurningEncoderReversed);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        turningPIDCon.enableContinuousInput(-1023, 1023);
    }

    public void forward(double sp) {
        System.out.println("SP: " + sp);
        /* m_turningMotor.set(TalonSRXControlMode.Position, unitConv(0));
        if (Math.abs(m_turningMotor.getSelectedSensorPosition() - unitConv(0)) < 10) {
            m_driveMotor.set(sp);
        }*/

        turningMotor.set(TalonSRXControlMode.Position, unitConv(0));
        if (sp > 0) { // forward
            //System.out.println("SENSOR: " + m_turningMotor.getSelectedSensorPosition());
            //sp = Math.abs(sp);
            //m_turningMotor.set(TalonSRXControlMode.Position, unitConv(180));
            if (Math.abs(turningMotor.getSelectedSensorPosition()-unitConv(0)) < 20) {
                driveMotor.set(sp);
                System.out.println("RX" + sp);
            }
            //System.out.println("CONV: " + unitConv(0));
        } else if (sp < 0) { // backward
            //System.out.println("SENSOR: " + m_turningMotor.getSelectedSensorPosition());

            //sp = Math.abs(sp);
            //m_turningMotor.set(TalonSRXControlMode.Position, unitConv(0));
            if (Math.abs(turningMotor.getSelectedSensorPosition()-unitConv(0)) < 20) {
                driveMotor.set(sp);
                System.out.println("LX: " + sp);
            }
            //System.out.println("CONV: " + unitConv(180));
        } else {
            resetPos(0.0);
        }
    }

    public void strafe(double d) {
        if (d > 0) { // right
            //System.out.println("SENSOR: " + m_turningMotor.getSelectedSensorPosition());
            d = Math.abs(d);
            turningMotor.set(TalonSRXControlMode.Position, unitConv(90));
            if (Math.abs(turningMotor.getSelectedSensorPosition()-unitConv(90)) < 20) {
                driveMotor.set(d);
                //System.out.println("RX" + d);
            }
            //System.out.println("CONV: " + unitConv(90));
        } else if (d < 0) { // left
            //System.out.println("SENSOR: " + m_turningMotor.getSelectedSensorPosition());

            d = Math.abs(d);
            turningMotor.set(TalonSRXControlMode.Position, unitConv(-90));
            if (Math.abs(turningMotor.getSelectedSensorPosition()-unitConv(-90)) < 20) {
                driveMotor.set(d);
                //System.out.println("LX: " + d);
            }
            //System.out.println("CONV: " + unitConv(-90));
        } else {
            resetPos(0.0);
        }
    }

    public void rotate(double deg, double sp) {
        //System.out.println("SP: " + sp);
        if (sp != 0) {
            turningMotor.set(TalonSRXControlMode.Position, unitConv(deg));

            if (Math.abs(turningMotor.getSelectedSensorPosition() - unitConv(deg)) < Constants204.Drivetrain.ROTATE_DRIVE_THRESHOLD) {
                driveMotor.set(sp);
            }
        } else {
          // resetPos(0.0);
        }
    }

    public void fullStrafe(PolarCoordinate pc) {
        if (Math.abs(pc.mag) > LX_MAG_DEADBAND) {
            //System.out.println("MAG:" + pc.mag + " DEG:" + pc.deg);
            turningMotor.set(TalonSRXControlMode.Position, unitConv(pc.deg));
            if (Math.abs(turningMotor.getSelectedSensorPosition()-unitConv(pc.deg)) < Constants204.Drivetrain.STRAFE_DRIVE_THRESHOLD) {
                driveMotor.set(pc.mag);
            }
        } else {
            //resetPos(pc.deg);
        }
    }

    public void setBrakeMode() {
        turningMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void turn(int deg) {
        if (Math.abs(turningMotor.getSelectedSensorPosition()-unitConv(deg)) < STRAFE_DRIVE_THRESHOLD) {
            //drivePIDCon.setReference(revs, CANSparkMax.ControlType.kPosition); // should replace all driveMotor.set with drivePIDcon.set(x, CSM.CT.kVelocity??)
        } else {
            turningMotor.set(TalonSRXControlMode.Position, unitConv(deg));
        }
        //System.out.println("units: " + driveEncoder.getPosition());
    }

    public void resetPos(double total) {
        //int revolutions = ((int)total)/360;
       // System.out.println("Target Degrees "+unitConv(360*revolutions));
        turningMotor.set(TalonSRXControlMode.Position, unitConv(total));
        
        driveMotor.set(0);
        //turningMotor.setSelectedSensorPosition(0);
       // return (double)360*revolutions;
    }

    public void setZero() {
        turningMotor.setSelectedSensorPosition(0);
        System.out.println("Sensor Position: "+ turningMotor.getSelectedSensorPosition());
    }

    public double unitConv(double d) {
        d = d / 360;
        return d * 1023;
    }

    public double getTurnEncDeg() {
        double deg = (turningMotor.getSelectedSensorPosition() / 1023) * 360;
        if (deg > 180) {
            deg = -(deg-180);
        }
        return deg;
    }
}
