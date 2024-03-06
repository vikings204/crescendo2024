package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Constants.Vision.*;

public class ShootSpeakerPoselessCommand extends Command {
    private final SwerveSubsystem Swerve;
    private final PoseEstimationSubsystem PoseEst;
    private final ShooterSubsystem Shooter;
    private double initialTimestamp;

    private final PIDController xPID = new PIDController(3, 0, 0);
    private final ProfiledPIDController thetaPID = new ProfiledPIDController(2, 0, 0, new Constraints(Constants.Swerve.MAX_ANGULAR_VELOCITY, 4));

    private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);
    private double range;

    public ShootSpeakerPoselessCommand(
            SwerveSubsystem Swerve,
            PoseEstimationSubsystem PoseEst,
            ShooterSubsystem Shooter
    ) {
        this.Swerve = Swerve;
        this.PoseEst = PoseEst;
        this.Shooter = Shooter;

        addRequirements(Swerve, Shooter);
    }

    @Override
    public void initialize() {
        initialTimestamp = getFPGATimestamp();
        Shooter.flywheelSpeaker(true);

        thetaPID.setGoal(new Rotation2d(Robot.alliance == DriverStation.Alliance.Blue ? 0 : 1).getRadians());
    }
    @Override
    public void execute() {
        Pose2d robotPose = PoseEst.getCurrentPose();
        var result = camera.getLatestResult();

        if (result.hasTargets() && result.getBestTarget().getFiducialId() == 8 || result.getBestTarget().getFiducialId() == 4) {
            range = PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_TO_ROBOT.getZ(),//CAMERA_HEIGHT_METERS,
                            Units.inchesToMeters(53.88),//TARGET_HEIGHT_METERS,
                            0,//CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getPitch()));

            if (getFPGATimestamp() > initialTimestamp + SECONDS_TO_SPIN_UP && xPID.atSetpoint() && thetaPID.atGoal()) {
                Shooter.intake(true, false);
            }
        }
        Swerve.drive(new Translation2d(-xPID.calculate(range), 0), thetaPID.calculate(robotPose.getRotation().getRadians()), true, false); // isOpenLoop differs from teleop
    }
    @Override
    public void end(boolean interrupted) {
        Shooter.flywheelSpeaker(false);
        Shooter.intake(false, false);
    }
}