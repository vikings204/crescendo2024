package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.concurrent.atomic.AtomicReference;

import static frc.robot.Constants.Vision.*;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {

    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<>();

    public PhotonRunnable() {
        this.photonCamera = new PhotonCamera(CAMERA_NAME);

        PhotonPoseEstimator photonPoseEstimator = null;

        var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // PV estimates will always be blue, they'll get flipped by robot thread
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        //noinspection ConstantValue
        if (photonCamera != null) {
            photonPoseEstimator = new PhotonPoseEstimator(
                    layout, POSE_STRATEGY, photonCamera, CAMERA_TO_ROBOT.inverse());
        }

        this.photonPoseEstimator = photonPoseEstimator;

        //Shuffleboard.getTab("camera").addCamera("raw", "webcam", "https://photonvision.local:1181").withSize(7, 7);
        Shuffleboard.getTab("camera").addCamera("processed", "webcam", "http://photonvision.local:1182/stream.mjpg").withSize(7, 7);
    }

    @Override
    public void run() {
        // Get AprilTag data
        if (photonPoseEstimator != null && photonCamera != null && !RobotState.isAutonomous()) {
            var photonResults = photonCamera.getLatestResult();
            if (photonResults.hasTargets()
                    && (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
                photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                    var estimatedPose = estimatedRobotPose.estimatedPose;
                    // Make sure the measurement is on the field
                    if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS_X
                            && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS_Y) {
                        atomicEstimatedRobotPose.set(estimatedRobotPose);
                    }
                });
            }
        }
    }

    /**
     * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
     * new estimate that hasn't been returned before.
     * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
     *
     * @return latest estimated pose
     */
    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }

}