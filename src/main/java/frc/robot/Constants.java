// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.1016;
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) kEncoderCPR;

        public static final double kPModuleTurningController = 1;

        public static final double kPModuleDriveController = 1;
    }


    public static final class Swerve {
        public static final double fastDriveSpeedMultiplier = 1.0;
        public static final double normalDriveSpeedMultiplier = 1.0;
        public static final double slowDriveSpeedMultiplier = 1.0;
        public static final int PIGEON2_ID = 10;
        //SET ME Each Run...
        public static final double robotOffset = 0.0;

        public static final double stickDeadband = 0.1;

        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(26.25);
        public static final double wheelBase = Units.inchesToMeters(26.25);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        //public static final double driveGearRatio =
        //   (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // 6.75:1
        public static final double driveGearRatio = 8.14;
        public static final double angleGearRatio = (150.0 / 7.0); //
        public static final SwerveDriveKinematics swerveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
                        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 10;
        public static final int driveContinuousCurrentLimit = 10;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.01;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 1.0;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor =
                (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 10.0; // meters per second  //4.5
        public static final double maxAngularVelocity = 8; // 11.5 // radians per second

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = true;
        public static final boolean angleInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 9;
            //public static final int canCoderID = 4;
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(42.58);
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(62.99);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            //public static final int canCoderID = 1;
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(296.67);
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(312.58);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            //public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(470.5);
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(204.7);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            //public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(296.67);
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(9.2);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 4; // 4
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; // 3
        public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = .3;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

}
  


