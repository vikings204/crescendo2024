package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.function.Supplier;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
import static frc.robot.Constants.Vision.FLIPPING_POSE;
import static frc.robot.Constants.Vision.VISION_ENABLED;

public class PoseEstimationSubsystem extends SubsystemBase {

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
     */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

    private final Supplier<Rotation2d> rotationSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();
    private final PhotonRunnable photonEstimator = new PhotonRunnable();
    private final Notifier photonNotifier = new Notifier(photonEstimator);

    private OriginPosition originPosition = kBlueAllianceWallRightSide;
    private boolean sawTag = false;

    private double[] stupidPose = new double[]{0, 0, 0};

    public PoseEstimationSubsystem(
            Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modulePositionSupplier) {

        this.rotationSupplier = rotationSupplier;
        this.modulePositionSupplier = modulePositionSupplier;

        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.SWERVE_KINEMATICS,
                rotationSupplier.get(),
                modulePositionSupplier.get(),
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);

        if (VISION_ENABLED) {
            // Start PhotonVision thread
            photonNotifier.setName("PhotonRunnable");
            photonNotifier.startPeriodic(0.02);
        }

        Shuffleboard.getTab("field").add("pose est field", field).withWidget(BuiltInWidgets.kField).withSize(8, 5);
        //Shuffleboard.getTab("main").addNumber("pose X", poseEstimator.getEstimatedPosition()::getX);
        //Shuffleboard.getTab("main").addNumber("pose Y", poseEstimator.getEstimatedPosition()::getY);
        //Shuffleboard.getTab("main").addNumber("gyro angle", poseEstimator.getEstimatedPosition().getRotation()::getDegrees);
        Shuffleboard.getTab("main").addNumber("pose X", () -> stupidPose[0]);
        Shuffleboard.getTab("main").addNumber("pose Y", () -> stupidPose[1]);
        Shuffleboard.getTab("main").addNumber("pose theta", () -> stupidPose[2]);
    }

    /**
     * Sets the alliance. This is used to configure the origin of the AprilTag map
     *
     * @param alliance alliance
     */
    public void setAlliance(Alliance alliance) {
        boolean allianceChanged = false;
        switch (alliance) {
            case Blue:
                allianceChanged = (originPosition == kRedAllianceWallRightSide);
                originPosition = kBlueAllianceWallRightSide;
                break;
            case Red:
                allianceChanged = (originPosition == kBlueAllianceWallRightSide);
                originPosition = kRedAllianceWallRightSide;
                break;
            default:
                // No valid alliance data. Nothing we can do about it
        }

        if (allianceChanged && sawTag) {
            // The alliance changed, which changes the coordinate system.
            // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
            // needs to be transformed to the new coordinate system.
            var newPose = flipAlliance(getCurrentPose());
            poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
        }
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors
        poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());

        if (VISION_ENABLED) {
            var visionPose = photonEstimator.grabLatestEstimatedPose();
            if (visionPose != null) {
                // New pose from vision
                sawTag = true;
                var pose2d = visionPose.estimatedPose.toPose2d();
                if (originPosition != kBlueAllianceWallRightSide) {
                    pose2d = flipAlliance(pose2d);
                }
                if (Robot.alliance == DriverStation.Alliance.Blue) {
                    pose2d = new Pose2d(pose2d.getTranslation(), pose2d.getRotation().plus(new Rotation2d(1*Math.PI)));//System.out.println("pose estimation yaw: " + pose2d.getRotation());
                }
                poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
            }
        }

        // Set the pose on the dashboard
        var dashboardPose = poseEstimator.getEstimatedPosition();
        if (originPosition == kRedAllianceWallRightSide) {
            // Flip the pose when red, since the dashboard field photo cannot be rotated
            dashboardPose = flipAlliance(dashboardPose);
        }
        field.setRobotPose(dashboardPose);

        //System.out.println(poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        //stupidPose = new double[]{dashboardPose.getX(), dashboardPose.getY(), dashboardPose.getRotation().getDegrees()};
        stupidPose = new double[]{dashboardPose.getX(), dashboardPose.getY(), dashboardPose.getRotation().getDegrees()};
        //System.out.println(dashboardPose.getX() + "     " + dashboardPose.getY());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     *
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is always on the right corner of your
     * alliance wall, so for 2023, the field elements are at different coordinates for each alliance.
     *
     * @param poseToFlip pose to transform to the other alliance
     * @return pose relative to the other alliance's coordinate system
     */
    private Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(FLIPPING_POSE);
    }

}