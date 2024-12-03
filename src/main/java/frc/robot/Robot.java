package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    public enum AutoMode {
        MidToTop("middle to top"),
        MidToBot("middle to bottom"),
        TopToTop("top to top"),
        BotToBot("bottom to bottom"),
        TopToEsc("top to escape"),
        BotToEsc("bottom to escape"),
        TopToEsc_Red("R top to escape"),
        BotToEsc_Red("R bottom to escape"),
        TopTwoNote_Red("R top two note"),
        MidToMid("middle to middle");

        public final String pathplannerName;
        AutoMode(String str) {
            this.pathplannerName = str;
        }

        private String decode(char[] chars) {
            String str = "" + chars[0] + chars[1] + chars[2];
            if (str.equals("Mid")) {
                return "Middle";
            } else if (str.equals("Bot")) {
                return "Bottom";
            } else if (str.equals("Esc")) {
                return "Escape";
            } else {
                return str;
            }
        }
        public String optionName() {
            return this.toString();
            //char[] chars = this.toString().toCharArray();
            //return "PP " + decode(new char[]{chars[0], chars[1], chars[2]}) + " TO " + decode(new char[]{chars[5], chars[6], chars[7]}) + (chars.length > 7 ? " RED" : " BLUE");
        }
    }
    public static final SendableChooser<ControlMode> ControlModeChooser = new SendableChooser<>();
    public static final SendableChooser<AutoMode> AutoModeChooser = new SendableChooser<>();

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
        //PortForwarder.add(8888, "10.2.4.69", 80);

        if (Constants.Controller.DEFAULT_CONTROL_MODE == ControlMode.SINGLE) {
            ControlModeChooser.setDefaultOption("Single Controller (Driver:usb1 Operator:usb1)", ControlMode.SINGLE);
            ControlModeChooser.addOption("Competition (Driver:usb1 Operator:usb2)", ControlMode.COMPETITION);
        } else if (Constants.Controller.DEFAULT_CONTROL_MODE == ControlMode.COMPETITION) {
            ControlModeChooser.addOption("Single Controller (Driver:usb1 Operator:usb1)", ControlMode.SINGLE);
            ControlModeChooser.setDefaultOption("Competition (Driver:usb1 Operator:usb2)", ControlMode.COMPETITION);
        }
        Shuffleboard.getTab("main").add("control mode", ControlModeChooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withSize(2, 1);

        for (AutoMode i : AutoMode.values()) {
            AutoModeChooser.addOption(i.optionName(), i);
        }

        Shuffleboard.getTab("main").add("Auto Select", AutoModeChooser).withSize(3, 1);
        checkDriverStationUpdate();
        Shuffleboard.getTab("main").addString("alliance", () -> allianceString);
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
        checkDriverStationUpdate();
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
        checkDriverStationUpdate();
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
        checkDriverStationUpdate();
    }
    @Override
    public void testPeriodic() {
    }

    public static DriverStation.Alliance alliance;
    public static String allianceString = "never init";
    /**
     * Checks the driverstation alliance. We have have to check repeatedly because we don't know when the
     * driverstation/FMS will connect, and the alliance can change at any time in the shop.
     */
    private void checkDriverStationUpdate() {
        // https://www.chiefdelphi.com/t/getalliance-always-returning-red/425782/27
        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();

        if (allianceOpt.isPresent()) {
            DriverStation.Alliance newAlliance = allianceOpt.get();
            robotContainer.PoseEstimation.setAlliance(DriverStation.Alliance.Blue);//robotContainer.PoseEstimation.setAlliance(newAlliance);
            alliance = newAlliance;
            allianceString = newAlliance.toString();
        }
    }
}
