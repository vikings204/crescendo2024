package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;


public class SwerveSubsystem extends SubsystemBase {
    public Pigeon2 gyro = new Pigeon2(Constants.Swerve.PIGEON2_ID, "rio");
    private SwerveDriveOdometry swerveOdometry; // Odometry class helps track where the robot is relative to where it started
    private SwerveModule[] modules; // Array of the 4 swerve modules
    private Field2d field;

    public SwerveSubsystem() {
        var toApply = new Pigeon2Configuration();
        gyro.getConfigurator().apply(toApply);

        zeroGyro();
        //m_gyro.calibrate();
        modules =
                new SwerveModule[]{
                        new SwerveModule(0, Constants.Swerve.Mod0.constants), //Each Constant set is specific to a motor pair
                        new SwerveModule(1, Constants.Swerve.Mod1.constants),
                        new SwerveModule(2, Constants.Swerve.Mod2.constants),
                        new SwerveModule(3, Constants.Swerve.Mod3.constants)
                };

        swerveOdometry =
                new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());

        //Odomentry with our kinematics object from constants, gyro position and x/y position of each module

         AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetOdometry, 
      this::getSpeeds, 
      this::driveRobotRelative, 
      Constants.Swerve.pathFollowerConfig,
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(fieldRelative ?
                        ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw()) :
                        new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false); // false
        }
    }

    public Pose2d getPose() {
        SmartDashboard.putNumber("pose X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("pose Y", swerveOdometry.getPoseMeters().getY());

        SmartDashboard.putNumber("gyro angle", gyro.getAngle());

        // SmartDashboard.putNumber("gyro filtered X", gyro.getXFilteredAccelAngle()); // loops between
        // about 14...0...360...346
        // SmartDashboard.putNumber("gyro filtered Y", gyro.getYFilteredAccelAngle()); // forward and
        // back leveling
        // 0-14, drive forward, 346-360 drive backward

        // SmartDashboard.putNumber("gyro pitch", gyro.getPitch());
        //SmartDashboard.putNumber("gyro roll", gyro.getRoll());
        //SmartDashboard.putNumber("pitch rate", getPitchRate());

        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }
    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
      }

      public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
      }
    
      public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
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
            SmartDashboard.putNumber(
                    "position: module " + mod.moduleNumber, mod.getPosition().distanceMeters);
            SmartDashboard.putNumber(
                    "angle: module " + mod.moduleNumber, mod.getPosition().angle.getDegrees());
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    public void zeroGyro(double yaw) {
        gyro.setYaw(yaw);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro)
                ? Rotation2d.fromDegrees(360 - gyro.getAngle())
                : Rotation2d.fromDegrees(gyro.getAngle());
    }

    public double getYawRate() {
        //return gyro.getRawGyroZ();
        //return m_gyro.getRate();
        return gyro.getRate();
    }

    // public double getXFilteredAccelAngle() {
    //   return gyro.getXFilteredAccelAngle();
    // }

    // public double getYFilteredAccelAngle() {
    //   return gyro.getYFilteredAccelAngle();
    // }

    public double getPitch() {
        return 0.0;//gyro.getPitch();
    }

    public double getPitchRate() {
        return 0.0;//gyro.getRawGyroY();
    }

    public double getRoll() {
        return 0.0;//gyro.getRoll();
    }

    
    // SmartDashboard.putNumber("gyro filtered X", gyro.getXFilteredAccelAngle()); // loops between
    // about 14...0...360...346
    // SmartDashboard.putNumber("gyro filtered Y", gyro.getYFilteredAccelAngle()); // forward and back
    // leveling
    // 0-14, drive forward, 346-360 drive backward

    public void setX() {
        modules[0].setAngleForX(45);
        modules[1].setAngleForX(-45);
        modules[2].setAngleForX(-45);
        modules[3].setAngleForX(45);
    }

    public void resetEncoders() {
        modules[0].resetEncoder();
        modules[1].resetEncoder();
        modules[2].resetEncoder();
        modules[3].resetEncoder();
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getPositions());
        field.setRobotPose(getPose());

        for (SwerveModule mod : modules) {
            //  SmartDashboard.putNumber(
            //    "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}
