package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import static frc.robot.Constants.Swerve.*;


public class SwerveSubsystem extends SubsystemBase {
    public Pigeon2 gyro = new Pigeon2(PIGEON2_ID, "rio");
    //private final SwerveDriveOdometry swerveOdometry; // Odometry class helps track where the robot is relative to where it started
    private final SwerveModule[] modules; // Array of the 4 swerve modules

    public SwerveSubsystem() {
        var toApply = new Pigeon2Configuration();
        gyro.getConfigurator().apply(toApply);

        zeroGyro();
        //m_gyro.calibrate();
        modules = new SwerveModule[]{
                        new SwerveModule(0, Mod0.DRIVE_MOTOR_ID, Mod0.ANGLE_MOTOR_ID, Mod0.ANGLE_OFFSET), //Each Constant set is specific to a motor pair
                        new SwerveModule(1, Mod1.DRIVE_MOTOR_ID, Mod1.ANGLE_MOTOR_ID, Mod1.ANGLE_OFFSET),
                        new SwerveModule(2, Mod2.DRIVE_MOTOR_ID, Mod2.ANGLE_MOTOR_ID, Mod2.ANGLE_OFFSET),
                        new SwerveModule(3, Mod3.DRIVE_MOTOR_ID, Mod3.ANGLE_MOTOR_ID, Mod3.ANGLE_OFFSET)
                };

        //Odomentry with our kinematics object from constants, gyro position and x/y position of each module
        //swerveOdometry = new SwerveDriveOdometry(SWERVE_KINEMATICS, getYaw(), getPositions());

//        AutoBuilder.configureHolonomic(
//                poseEstimator::getCurrentPose,//this::getPose,
//                this::resetOdometry,
//                this::getSpeeds,
//                this::driveRobotRelative,
//                Constants.Auto.PATH_FOLLOWER_CONFIG,
//                () -> {
//                    // Boolean supplier that controls when the path will be mirrored for the red alliance
//                    // This will flip the path being followed to the red side of the field.
//                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
//
//                    var alliance = DriverStation.getAlliance();
//                    //noinspection OptionalIsPresent
//                    if (alliance.isPresent()) {
//                        return alliance.get() == DriverStation.Alliance.Red;
//                    }
//                    return false;
//                },
//                this
//        );
        
        for (SwerveModule mod : modules) {
            Shuffleboard.getTab("swerve").addNumber("position: module " + mod.moduleNumber, () -> mod.getPosition().distanceMeters);
            Shuffleboard.getTab("swerve").addNumber("angle: module " + mod.moduleNumber, mod.getPosition().angle::getDegrees).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -180, "max", 180));
            Shuffleboard.getTab("swerve").addNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle::getDegrees).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -180, "max", 180));
            Shuffleboard.getTab("swerve").addNumber("Mod " + mod.moduleNumber + " Velocity", () -> mod.getState().speedMetersPerSecond);
        }
        Shuffleboard.getTab("main").addNumber("gyro angle", () -> {
            Rotation2d yaw = getYaw();
            return yaw.getDegrees();
        });
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                SWERVE_KINEMATICS.toSwerveModuleStates(fieldRelative ?
                        ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw()) :
                        new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        //System.out.println("Current Heading: "+getYaw());
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false); // false
        }
    }

//    public Pose2d getPose() {
//        SmartDashboard.putNumber("pose X", swerveOdometry.getPoseMeters().getX());
//        SmartDashboard.putNumber("pose Y", swerveOdometry.getPoseMeters().getY());
//
//        SmartDashboard.putNumber("gyro angle", gyro.getAngle());
//
//        // SmartDashboard.putNumber("gyro filtered X", gyro.getXFilteredAccelAngle()); // loops between
//        // about 14...0...360...346
//        // SmartDashboard.putNumber("gyro filtered Y", gyro.getYFilteredAccelAngle()); // forward and
//        // back leveling
//        // 0-14, drive forward, 346-360 drive backward
//
//        // SmartDashboard.putNumber("gyro pitch", gyro.getPitch());
//        //SmartDashboard.putNumber("gyro roll", gyro.getRoll());
//        //SmartDashboard.putNumber("pitch rate", getPitchRate());
//
       // return swerveOdometry.getPoseMeters();
   // }

//    public void resetOdometry(Pose2d pose) {
//        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
//    }

    public ChassisSpeeds getSpeeds() {
        return SWERVE_KINEMATICS.toChassisSpeeds(getStates());
    }

//    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
//        //driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
//        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, poseEstimator.getCurrentPose().getRotation()));
//    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = SWERVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : modules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : modules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    public void setGyro(double yaw) {
        gyro.setYaw(yaw);
    }

    public Rotation2d getYaw() {
        return (GYRO_INVERT)
                ? Rotation2d.fromDegrees(360 - gyro.getAngle())
                : Rotation2d.fromDegrees(gyro.getAngle());
    }

//    public double getYawRate() {
//        //return gyro.getRawGyroZ();
//        //return m_gyro.getRate();
//        return gyro.getRate();
//    }

    // public double getXFilteredAccelAngle() {
    //   return gyro.getXFilteredAccelAngle();
    // }

    // public double getYFilteredAccelAngle() {
    //   return gyro.getYFilteredAccelAngle();
    // }

//    public double getPitch() {
//        return 0.0;//gyro.getPitch();
//    }

//    public double getPitchRate() {
//        return 0.0;//gyro.getRawGyroY();
//    }

//    public double getRoll() {
//        return 0.0;//gyro.getRoll();
//    }
public void resetEncoders(){
    for (int i = 0; i<4; i++){
        modules[i].resetToAbsolute();
      }
}

//    public void resetEverything() {
////        modules[0].setAngleForX(0);
////        modules[1].setAngleForX(0);
////        modules[2].setAngleForX(0);
////        modules[3].setAngleForX(0);
//
//        modules[0].resetEncoder();
//        modules[1].resetEncoder();
//        modules[2].resetEncoder();
//        modules[3].resetEncoder();
//
//        resetOdometry(new Pose2d());
//        gyro.reset();
//    }

    @Override
    public void periodic() {
        //swerveOdometry.update(getYaw(), getPositions());
        //field.setRobotPose(getPose());

        // for (SwerveModule mod : modules) {
        //     //  SmartDashboard.putNumber(
        //     //    "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        //     Shuffleboard.getTab("swerve").addNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle::getDegrees).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -180, "max", 180)); // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
        //     Shuffleboard.getTab("swerve").addNumber("Mod " + mod.moduleNumber + " Velocity", () -> mod.getState().speedMetersPerSecond); //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        // }
    }
}
