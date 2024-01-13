package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants204.Drivetrain;
import frc.lib.config.CTREConfigs;
import java.util.concurrent.TimeUnit;
import com.pathplanner.lib.util.*;
//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.server.PathPlannerServer;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs;

    private Command m_autonomousCommand;
    private Command teleopCommand;
    private static final double kAngleSetpoint = 0.0;
	private static final double kP = 0.005;

    private RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        ctreConfigs = new CTREConfigs();
  
        //m_gyro.getAngle()
        robotContainer = new RobotContainer();
        //PathPlannerServer.startServer(5812);
        //CameraServer.startAutomaticCapture(); // use for USB camera
        PortForwarder.add(8888, "10.2.4.69", 80);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        if (teleopCommand != null) {
            teleopCommand.cancel();
        }

      //  m_autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
          m_autonomousCommand.schedule();
        }


//        try {
//            TimeUnit.MILLISECONDS.sleep(Constants204.Automation.DRIVE_BACKWARD_MS);
//        } catch (InterruptedException e) {
//            System.out.println("FAILED TO WAIT FOR SOME FUCKING REASON");
//        }
//        robotContainer.autoStateMachine++;
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

  
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        //teleopCommand.cancel();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        //System.out.println(robotContainer.strafeDrive.TestEncoders());
       // robotContainer.strafeDrive.moreDrive(0, 0, 0);
        if (robotContainer.CONTROLLER.getYButton()) {
            //robotContainer.strafeDrive.setZero();
            Translation2d t1 = new Translation2d(0,0);
            Rotation2d r1 = new Rotation2d(0);

            Pose2d pstart = new Pose2d(t1,r1);
            robotContainer.s_Swerve.resetOdometry(pstart);
            //robotContainer.strafeDrive.turningTotalDeg = 0.0;
            System.out.println("You have 0'd the turning encoders");
            robotContainer.autoStateMachine = 0;

 
        }
        if (robotContainer.CONTROLLER.getAButton()) {
            //robotContainer.armControl.boomStart= robotContainer.armControl.boomEncoder.getPosition();
            //robotContainer.armControl.dipperMax= robotContainer.armControl.dipperEncoder.getPosition();
            //System.out.println("Boom Start is Now: "+ robotContainer.armControl.boomStart+"\n Dipper Max is Now: "+robotContainer.armControl.dipperMax);

        }
        if (robotContainer.CONTROLLER.getBButton()) {

        }
        if (robotContainer.CONTROLLER.getXButton()) {

        }

        double armB=0.0, armD=0.0, armC=0.0;
        if (robotContainer.CONTROLLER.getRightUpperBumper()) { armB = -1; } else if (robotContainer.CONTROLLER.getRightTriggerAxis()>0.2) { armB = 1; }
        if (robotContainer.CONTROLLER.getLeftUpperBumper()) { armD = -1; } else if (robotContainer.CONTROLLER.getLeftTriggerAxis()>0.2) { armD = 1; }
        if (robotContainer.CONTROLLER.getBButton()) { armC = 1; } else if (robotContainer.CONTROLLER.getXButton()) { armC = -1; }
        //robotContainer.armControl.setArmTest(armB, armD, armC);
        
    }
}
