package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Optional;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public enum ControlMode {
        SINGLE, COMPETITION
    }
    public static final SendableChooser<ControlMode> ControlModeChooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.

        robotContainer = new RobotContainer();
        //CameraServer.startAutomaticCapture(); // use for USB camera
        PortForwarder.add(8888, "10.2.4.69", 80);

        //ControlModeChooser.setDefaultOption("Single Controller (Driver:usb1 Operator:usb1)", ControlMode.SINGLE);
        //ControlModeChooser.addOption("Competition (Driver:usb1 Operator:usb2)", ControlMode.COMPETITION);
        ControlModeChooser.addOption("Single Controller (Driver:usb1 Operator:usb1)", ControlMode.SINGLE);
        ControlModeChooser.setDefaultOption("Competition (Driver:usb1 Operator:usb2)", ControlMode.COMPETITION);
        SmartDashboard.putData("Control Mode", ControlModeChooser);
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



    @Override
    public void disabledInit() {
    }
    @Override
    public void disabledPeriodic() {
    }



    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
          autonomousCommand.schedule();
        }
    }
    @Override
    public void autonomousPeriodic() {
    }



    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        checkDriverStationUpdate();

    }
    @Override
    public void teleopPeriodic() {
    }



    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        //teleopCommand.cancel();
    }
    @Override
    public void testPeriodic() {
    }

    public static DriverStation.Alliance alliance;
    /**
     * Checks the driverstation alliance. We have have to check repeatedly because we don't know when the
     * driverstation/FMS will connect, and the alliance can change at any time in the shop.
     */
    private void checkDriverStationUpdate() {
        // https://www.chiefdelphi.com/t/getalliance-always-returning-red/425782/27
        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();

        if (allianceOpt.isPresent()) {
            DriverStation.Alliance newAlliance = allianceOpt.get();
            // If we have data, and have a new alliance from last time
            if (DriverStation.isDSAttached() && newAlliance != alliance) {
                robotContainer.PoseEstimation.setAlliance(newAlliance);
                alliance = newAlliance;
            }
        }
    }
}
