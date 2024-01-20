package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.util.*;
//import frc.robot.Auto.*;

/*import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;*/

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final SwerveSubsystem SwerveSubsystem = new SwerveSubsystem();
    private final AutonomousManager AutonomousManager = new AutonomousManager(SwerveSubsystem);
    Gamepad CONTROLLER = new Gamepad(Constants204.Controller.PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        SwerveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        SwerveSubsystem,
                        () -> CONTROLLER.getLeftX(),
                        () -> -1 * CONTROLLER.getLeftY(),
                        () -> -1 * CONTROLLER.getRightX(),
                        () -> false,
                        () -> CONTROLLER.getYButton(),
                        () -> CONTROLLER.getAButton()));


        if (CONTROLLER.getBButton()) {
            SwerveSubsystem.gyro.reset();
            // s_Swerve.m_gyro_P2.calib;
            System.out.println("you have calibed the gyro");
        }
    }

    public Command getAutonomousCommand() {
        return AutonomousManager.getCommand();
    }
}
