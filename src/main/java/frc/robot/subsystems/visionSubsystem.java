package frc.robot.subsystems;
import java.util.List;
import java.util.Optional;
import edu.wpi.first.wpilibj.XboxController;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class visionSubsystem extends SubsystemBase {
    public static XboxController controller = new XboxController(5);
    public static PhotonCamera camera = new PhotonCamera("Camera205");
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public visionSubsystem(){

    //Basic Camera Data to be used
    }
    public static boolean hasTargets(){
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        System.out.println(hasTargets);
        return hasTargets;
    }
     public static List<PhotonTrackedTarget> targets(){
        var result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        System.out.println(targets);
        return targets;
    }
     public static  PhotonTrackedTarget getTarget(){
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        System.out.println(target);
        return target;
    }
    public static int getTargetID(){
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        int targetID = target.getFiducialId();
        System.out.println(targetID);
        return targetID = target.getFiducialId();
    }
    public static void getPoseAmbiguity(){
        if(visionSubsystem.hasTargets()){
            var result = camera.getLatestResult();
            PhotonTrackedTarget target = result.getBestTarget();
            double poseAmbiguity = target.getPoseAmbiguity();
            System.out.println(poseAmbiguity);
        }
    }
    public static Transform3d getBestCameraToTarget(){
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        System.out.println(bestCameraToTarget);
        return bestCameraToTarget;
    }
     public static Transform3d getAlternateCameraToTarget(){
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
        System.out.println(alternateCameraToTarget);
        return alternateCameraToTarget;
    }
    public static double getYaw(){
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        double yaw = target.getYaw();
        System.out.println(yaw);
        return yaw;
    }
    public static double getPitch(){
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        double pitch = target.getPitch();
        System.out.println(pitch);
        return pitch;
    }
    public static double getArea(){
         var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        double area = target.getArea();
        System.out.println(area);
        return area;
    }
    
    public static double getSkew(){
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        double skew = target.getSkew();
        System.out.println(skew);
        return skew;
    }
    public static List<TargetCorner> getCorners(){
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        List<TargetCorner> corners = target.getDetectedCorners();
        System.out.println(corners);
        return corners;    }
   // Let the robot find where it is
    public static Pose3d getRobotPose(){
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d cameraToRobot = new Transform3d();

        
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),getAprilTagPose(), cameraToRobot);
        System.out.println(robotPose);
        return robotPose;
    }
    public void returnAll(){
       if(hasTargets()){
        System.out.println(getRobotPose());
       }
    }
    public static Pose3d getAprilTagPose(){
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        Optional<Pose3d> temp = aprilTagFieldLayout.getTagPose(target.getFiducialId());
        Pose3d aprilTagPose = temp.get(); 
        return aprilTagPose;
        

    }
    //I have no idea if the below works.
    // before running this, we need a function that sets the target height variable depending on which tag id it has, tags on the field vary in height from ~3-4ft.
    //get the height of a tag from carpet based on its ID, used by getDistanceToTarget; sourced by Crescendo manual
    public static double getTagHeight(int targetId){
        //returned in meters
        switch(targetId) {
            //SOURCE apriltags
            case 1:
                return 1.22;
            case 2:
                return 1.22;
            case 9:
                return 1.22;
            case 10:
                return 1.22;
            default:
              // code block
          }


        return 1.1;
    }
    public static double getDistanceToTarget(){
        double range = 0;
        if (controller.getAButton()) {
           
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();

            if (result.hasTargets()) {
                // First calculate range
                range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                0.25,// NOTE: MEASURE AND CHANGE FOR ACTUAL RELEASE 
                                1.2,
                                bootlegDegreesToRadiansUnilIfigureOutHowUnitsClassWorks(31.5),
                               
                                bootlegDegreesToRadiansUnilIfigureOutHowUnitsClassWorks(getPitch())
                        );

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                //forwardSpeed = -controller.calculate(range, GOAL_RANGE_METERS);
                
           }
        }
        return range;
    }
    public static double bootlegDegreesToRadiansUnilIfigureOutHowUnitsClassWorks(double degrees){
        return degrees * (Math.PI/180);

    }
}
