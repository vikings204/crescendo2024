package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot.ControlMode;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.util.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import static frc.robot.Robot.ControlModeChooser;
import static frc.robot.Robot.ControlMode;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final SwerveSubsystem SwerveSubsystem = new SwerveSubsystem();
    public final ShooterSubsystem ShooterSubsystem = new ShooterSubsystem();
    public final LinearActuatorSubsystem LinearActuatorSubsystem = new LinearActuatorSubsystem();
    private final AutonomousManager AutonomousManager = new AutonomousManager(SwerveSubsystem);
    Gamepad DRIVER = new Gamepad(Constants204.Controller.DRIVER_PORT);
    Gamepad OPERATOR = new Gamepad(Constants204.Controller.DRIVER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        ControlModeChooser.onChange((ControlMode mode) -> {
            if (mode == ControlMode.SINGLE) {
                OPERATOR = new Gamepad(Constants204.Controller.DRIVER_PORT);
            } else {
                OPERATOR = new Gamepad(Constants204.Controller.OPERATOR_PORT);
            }
            configureButtonBindings();
            System.out.println("Switched control mode to " + mode);
        });

        configureButtonBindings();
        SwerveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        SwerveSubsystem,
                        () -> DRIVER.getLeftX(),
                        () -> -1 * DRIVER.getLeftY(),
                        () -> -1 * DRIVER.getRightX(),
                        () -> false,
                        () -> DRIVER.getYButton(),
                        () -> DRIVER.getAButton()));


        if (DRIVER.getBButton()) {
            SwerveSubsystem.gyro.reset();
            // s_Swerve.m_gyro_P2.calib;
            System.out.println("you have calibed the gyro");
        }

        ShooterSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> ShooterSubsystem.speakerShot(OPERATOR.getXButton()),
                        ShooterSubsystem));

        LinearActuatorSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> LinearActuatorSubsystem.shift(OPERATOR.getPOV()==0, OPERATOR.getPOV()==180),
                        LinearActuatorSubsystem
                )
        );
    }
    private void configureButtonBindings() {
        new JoystickButton(OPERATOR, 5)
        .whileTrue(
            new RunCommand(() -> ShooterSubsystem.receive(true), ShooterSubsystem));

        new JoystickButton(OPERATOR, 6)
        .whileTrue(
            new RunCommand(() -> ShooterSubsystem.bump(true), ShooterSubsystem));    

         new JoystickButton(OPERATOR, 10)
        .whileTrue(
            new RunCommand(() -> ShooterSubsystem.ampShot(true), ShooterSubsystem));    
            
    }
        public Command getAutonomousCommand() {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Path 1");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPathWithEvents(path);}

    /*public Command getAutonomousCommand() {
        return AutonomousManager.getCommand();
    }*/
}
