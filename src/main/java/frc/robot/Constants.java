package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Servo;

import com.revrobotics.CANSparkBase.IdleMode;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import org.photonvision.PhotonPoseEstimator;

import static edu.wpi.first.math.util.Units.inchesToMeters;

@SuppressWarnings("unused")
public final class Constants {
    public static final class Controller {
        public static final int DRIVER_PORT = 1;
        public static final int OPERATOR_PORT = 2;
        public static final double DEADBAND = 0.1;

        // will eventually have keybinds and stuff
    }

    public static final class Shooter {
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;
        public static final boolean SHOOTER_INVERT = true;
        public static final boolean BUMP_INVERT = false;

        public static final int SHOOTER_MOTOR1_ID = 41;
        public static final int SHOOTER_MOTOR_2_ID = 42;
        public static final int INTAKE_MOTOR_ID = 43;

        public static final int CURRENT_LIMIT = 40;
        public static final double VOLTAGE_COMP = 16.0;

        public static final double SPEAKER_SPEED = 1.0;
        public static final double AMP_SPEED = .080;
        public static final double INTAKE_SPEED = 1.0;

        public static final int INTAKE_SENSOR_THRESHOLD = 200;
    }

    public static final class Swerve {
        public static final double FAST_SPEED_MULTIPLIER = 1;
        public static final double NORMAL_SPEED_MULTIPLIER = 1;//.8;
        public static final double SLOW_SPEED_MULTIPLIER = .6;

        public static final double ANGLE_PID_FF = 0.0;
        public static final double DRIVE_PID_P = 1.0;
        public static final double DRIVE_PID_I = 0.0;
        public static final double DRIVE_PID_D = 0.01;
        public static final double DRIVE_PID_FF = 0.0;
        public static final double DRIVE_FF_S = 0.667;
        public static final double DRIVE_FF_V = 2.44;
        public static final double DRIVE_FF_A = 0.27;
        public static final double ANGLE_PID_P = 0.01;
        public static final double ANGLE_PID_I = 0.0;
        public static final double ANGLE_PID_D = 0.0;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = inchesToMeters(23); // same as wheelbase because it is a square
        public static final double WHEEL_BASE = inchesToMeters(23);
        public static final double WHEEL_DIAMETER = inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double DRIVE_GEAR_RATIO = 8.14;
        public static final double ANGLE_GEAR_RATIO = (150.0 / 7.0);
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        );

        /* Drive Motor Conversion Factors */
        public static final double DRIVE_POSITION_CONVERSION_FACTOR = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
        public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR / 60.0;
        public static final double ANGLE_POSITION_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

        /* Swerve Voltage Compensation */
        public static final double VOLTAGE_COMPENSATION = 12.0;

        /* Swerve Current Limiting */
        public static final int DRIVE_CURRENT_LIMIT = 40;//30;
        public static final int ANGLE_CURRENT_LIMIT = 10;//5;

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 4.5; // meters per second
        public static final double MAX_ANGULAR_VELOCITY = 8; // radians per second

        /* Neutral Modes */
        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean DRIVE_INVERT = true;
        public static final boolean ANGLE_INVERT = true;

        public static final boolean GYRO_INVERT = true; // Always ensure Gyro is CCW+ CW-
        public static final int PIGEON2_ID = 9;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 21;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(348.10);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 12;
            public static final int ANGLE_MOTOR_ID = 22;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(29.45);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 10;
            public static final int ANGLE_MOTOR_ID = 20;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(185.38);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 13;
            public static final int ANGLE_MOTOR_ID = 23;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(173.345);
        }
    }

    public static final class Auto {
        public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
                new PIDConstants(2.0, 0, .04), // Translation constants
                new PIDConstants(3.0, 0.1, .00), // Rotation constants
                Swerve.MAX_SPEED,
                .495, // Drive base radius (distance from center to the furthest module)
                new ReplanningConfig()
        );
    }

    public static final class LinearActuator {
        public static final int MOTOR_CAN_ID = 31;
        public static final int CURRENT_LIMIT = 10;

        public static final double PID_P = 1.0;
        public static final double PID_I = 0.0;
        public static final double PID_D = 0.0;
        public static final double PID_FF = 0.0;

        // minimum/maximum of the linear actuator, not the whole mechanism
        public static final double ABSOLUTE_LOWEST = -100000.0;
        public static final double ABSOLUTE_HIGHEST = 10000.0;

        // positions are of the entire mechanism
        public enum Position {
            EXAMPLE(0.0),
            ANOTHER_EXAMPLE(0.0);

            public final double position;
            Position(double p) {
                this.position = p;
            }
        }
    }

    public static final class Vision {
        public static final boolean VISION_ENABLED = false;
        public static final String CAMERA_NAME = "webcam";

        public static final PhotonPoseEstimator.PoseStrategy POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
        public static final double FIELD_LENGTH_METERS_X = 16.54175;
        public static final double FIELD_WIDTH_METERS_Y = 8.0137;
        public static final Pose2d FLIPPING_POSE = new Pose2d(
                new Translation2d(FIELD_LENGTH_METERS_X, FIELD_WIDTH_METERS_Y),
                new Rotation2d(Math.PI)
        );

        public static final double TARGET_OFFSET = inchesToMeters(50);//inchesToMeters(4);
        public static final Translation2d SPEAKER_RED = new Translation2d(inchesToMeters(652.73) - TARGET_OFFSET, inchesToMeters(218.42));
        //public static final Translation2d SPEAKER_BLUE = new Translation2d(TARGET_OFFSET, inchesToMeters(218.42));
        public static final Translation2d SPEAKER_BLUE = new Translation2d(2, 4.6);
    }
    public static final class Flap{
        public static final int FLAP_PWM_CHANNEL =1;
        public static final double AMP_POSITION = 0.5;
        public static final double SOURCE_POSITION = .5;
         public static final double CLOSED_POSITION = 0.02;
    }
}
  


