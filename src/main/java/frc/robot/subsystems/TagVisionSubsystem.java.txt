package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import static frc.robot.Constants204.Vision.*;

import java.util.Objects;

public class TagVisionSubsystem extends SubsystemBase {
    private static PhotonCamera pv;
    private int lcID, mcID, rcID, dsID; // leftcubeID, middlecubeID, rightcubeID, doublesubstationID
    private String currentAlliance = "red";

    public TagVisionSubsystem() {
        pv = new PhotonCamera(PHOTONVISION_NAME);

        pv.setDriverMode(false);
        pv.setPipelineIndex(1);
        switchAlliance(); // sets to blue
    }

    public String switchAlliance() {
        if (currentAlliance.equals("blue")) {
            // set to red AT ids
            lcID = 3;
            mcID = 2;
            rcID = 1;
            dsID = 5;
            currentAlliance = "red";
        } else if (currentAlliance.equals("red")) {
            // set to blue AT ids
            lcID = 8;
            mcID = 7;
            rcID = 6;
            dsID = 4;
            currentAlliance = "blue";
        } else {
            System.out.println("VISION ERROR: ALLIANCE STRING FUCKED UP - " + currentAlliance);
        }
        return currentAlliance;
    }

    @Override
    public void periodic() {
        // runs every scheduler loop (every ~20ms)
        //System.out.println("PV-AT_ID: " + getBestTranslation());
    }

    public String getLatestID() {
        PhotonPipelineResult r = pv.getLatestResult();
        if (r.hasTargets()) {
            return "PV-AT_ID: " + r.getBestTarget().getFiducialId();
        } else {
            return "PV-AT-NULL";
        }
    }

    public String getBestTranslation() {
        PhotonPipelineResult r = pv.getLatestResult();
        if (r.hasTargets()) {
            double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            Units.degreesToRadians(CAMERA_PITCH_DEGREES),
                            Units.degreesToRadians(r.getBestTarget().getPitch()));

            Translation2d trans = PhotonUtils.estimateCameraToTargetTranslation(
                    distanceMeters, Rotation2d.fromDegrees(-r.getBestTarget().getYaw()));

            return "id=" + r.getBestTarget().getFiducialId() + " distance=" + distanceMeters + " x=" + trans.getX() + " y=" + trans.getY() + "" + trans.getAngle();
        } else {
            return "null";
        }
    }

    public Translation2d getTranslation() {
        PhotonPipelineResult r = pv.getLatestResult();
        if (r.hasTargets()) {
            double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS,
                    TARGET_HEIGHT_METERS,
                    Units.degreesToRadians(CAMERA_PITCH_DEGREES),
                    Units.degreesToRadians(r.getBestTarget().getPitch()));

            Translation2d trans = PhotonUtils.estimateCameraToTargetTranslation(
                    distanceMeters, Rotation2d.fromDegrees(-r.getBestTarget().getYaw()));

            return trans;
        } else {
            return null;
        }
    }

    public Transform3d getTransform() {
        PhotonPipelineResult r = pv.getLatestResult();
        if (r.hasTargets()) {
            return r.getBestTarget().getBestCameraToTarget();
        } else {
            return null;
        }
    }
}
