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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.util.InterpLUT;
import frc.robot.Constants204;
import static frc.robot.Constants204.Drivetrain.*;


public class SingleStrafeSubsystem extends SubsystemBase {
  private final CANSparkMax m_driveMotor = new CANSparkMax(Constants204.DrivetrainCAN.SINGLE_STRAFE_DRIVE_MOTOR, MotorType.kBrushless);
  public final TalonSRX m_turningMotor = new TalonSRX(Constants204.DrivetrainCAN.SINGLE_STRAFE_TURNING_MOTOR);
  private final RelativeEncoder m_driveEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));


  public SingleStrafeSubsystem() {
    m_turningMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0 ); //Set the feedback device that is hooked up to the talon

    m_turningMotor.setSelectedSensorPosition(0);

    m_turningMotor.config_kP(0, STRAFE_TURNING_PID_P);
    m_turningMotor.config_kI(0, STRAFE_TURNING_PID_I);
    m_turningMotor.config_kD(0, STRAFE_TURNING_PID_D);
    
    m_turningMotor.setNeutralMode(NeutralMode.Brake);

    m_driveEncoder = m_driveMotor.getEncoder();

    m_turningMotor.configSelectedFeedbackCoefficient(1);

    // Set whether turning encoder should be reversed or not
    //m_turningMotor.setInverted(Constants.DriveConstants.kFrontLeftTurningEncoderReversed);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.enableContinuousInput(-1023, 1023);
  

  }

  public void strafe(double x) {
    //System.out.println("CALC: " + Math.abs(d));
    //y = Math.abs(y);
    if (x > 0.2) { // right
      System.out.println("SENSOR: " + m_turningMotor.getSelectedSensorPosition());
      x = Math.abs(x);
      m_turningMotor.set(TalonSRXControlMode.Position, unitConv(90));
      if (Math.abs(m_turningMotor.getSelectedSensorPosition()-unitConv(90)) < 10) {
        m_driveMotor.set(x);
        System.out.println("RX" + x);
      }
      System.out.println("CONV: " + unitConv(90));
    } else if (x < -0.2) { // left
      System.out.println("SENSOR: " + m_turningMotor.getSelectedSensorPosition());

      x = Math.abs(x);
      m_turningMotor.set(TalonSRXControlMode.Position, unitConv(-90));
      if (Math.abs(m_turningMotor.getSelectedSensorPosition()-unitConv(-90)) < 10) {
        m_driveMotor.set(x);
        System.out.println("LX: " + x);
      }
      System.out.println("CONV: " + unitConv(-90));
    } else {
      m_turningMotor.set(TalonSRXControlMode.Position, unitConv(0));
      m_driveMotor.set(0);
    }
  }

  public void setZero() {
    m_turningMotor.setSelectedSensorPosition(0);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningMotor.getSelectedSensorPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getCountsPerRevolution(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    //final double turnOutput = m_turningPIDController.calculate(m_turningMotor.getSelectedSensorPosition(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(TalonSRXControlMode.Position, unitConv(state.angle.getDegrees()));
    //m_turningMotor.set(TalonSRXControlMode.Position, m_turningPIDController.calculate(m_turningMotor.getSelectedSensorPosition(), unitConv(state.angle.getDegrees())));
    System.out.println("SENSOR: " + m_turningMotor.getSelectedSensorPosition());
    System.out.println("DEGREES: " + state.angle.getDegrees());
    System.out.println("CALCD: " + unitConv(state.angle.getDegrees()));
  }

  public double unitConv(double d) {
    d = d / 360;
    return d * 1023;

    /*if (d > 0) {
      d = d / 360;
      return d * 1023;
    } else {
      d = d / 360;
      d = d * 1023;
      return d + 1023;
    }*/
  }
}
