package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Optional;

// autonomous path planning this year relies on Choreo
// download planning app from here: https://github.com/SleipnirGroup/Choreo/releases/latest
// it is similar to pathplanner but the splines are automatically
// configured for the most efficient route while avoiding obstacles

public class AutonomousManager {
    private final SwerveSubsystem SwerveSubsystem = new SwerveSubsystem();
    private ChoreoTrajectory trajectory;

    public Command getCommand() {
        trajectory = Choreo.getTrajectory(Constants204.Autonomous.CHOREO_PATH_FILE);

        var thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveSubsystem.resetOdometry(trajectory.getInitialPose());

        Command swerveCommand = Choreo.choreoSwerveCommand(
                trajectory, // Choreo trajectory from above
                SwerveSubsystem::getPose, // A function that returns the current field-relative pose of the robot: your
                // wheel or vision odometry
                new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
                // translation (input: X error in meters,
                // output: m/s).
                new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
                // translation (input: Y error in meters,
                // output: m/s).
                thetaController, // PID constants to correct for rotation
                // error
                /*(ChassisSpeeds speeds) -> SwerveSubsystem.drive( // needs to be robot-relative
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond,
                        false),*/
                (ChassisSpeeds speeds) -> SwerveSubsystem.drive(
                        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                        speeds.omegaRadiansPerSecond,
                        false, // robot oriented swerve
                        true),
                this::isRedAlliance, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
                SwerveSubsystem // The subsystem(s) to require, typically your drive subsystem only
        );

        return Commands.sequence(
                Commands.runOnce(() -> SwerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveCommand,
                SwerveSubsystem.run(() -> SwerveSubsystem.drive(new Translation2d(0, 0), 0, false, false))
        );
    }

    private boolean isRedAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            DriverStation.Alliance a = alliance.get();
            if (a == DriverStation.Alliance.Red) {
                return true;
            } else if (a == DriverStation.Alliance.Blue) {
                return false;
            }
        }
        return false;
    }
}