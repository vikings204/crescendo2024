package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.NamedCommands.*;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot.ControlMode;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.LinearActuatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Gamepad;

import java.util.List;

import static frc.robot.Robot.ControlModeChooser;

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


        NamedCommands.registerCommand("intakeStart", new InstantCommand(() -> ShooterSubsystem.receive(true), ShooterSubsystem));
        NamedCommands.registerCommand("intakeStop", new InstantCommand(() -> ShooterSubsystem.receive(false), ShooterSubsystem));
        NamedCommands.registerCommand("shooterStart", new InstantCommand(() -> ShooterSubsystem.speakerShot(true), ShooterSubsystem));
        NamedCommands.registerCommand("shooterStop", new InstantCommand(() -> ShooterSubsystem.speakerShot(false), ShooterSubsystem));
        NamedCommands.registerCommand("bumpStart", new InstantCommand(() -> ShooterSubsystem.bump(true), ShooterSubsystem));
        NamedCommands.registerCommand("bumpStop", new InstantCommand(() -> ShooterSubsystem.bump(false), ShooterSubsystem));


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

        ShooterSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> ShooterSubsystem.speakerShot(OPERATOR.getXButton()),
                        ShooterSubsystem));

        LinearActuatorSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> LinearActuatorSubsystem.shift(OPERATOR.getPOV() == 0, OPERATOR.getPOV() == 180),
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

        new JoystickButton(DRIVER, 2)
                .whileTrue(
                        new RunCommand(SwerveSubsystem::resetEverything, SwerveSubsystem));

    }

    public Command getAutonomousCommand() {
        SwerveSubsystem.gyro.setYaw(-90.0); // temp for auto testing
        return new PathPlannerAuto("Intake Auto");
    }

//    public Command getAutonomousCommand() {
//        // Load the path you want to follow using its name in the GUI
//        PathPlannerPath path = PathPlannerPath.fromPathFile("Path 2");
//
//        // Create a path following command using AutoBuilder. This will also trigger event markers.
//        //noinspection removal
//        return AutoBuilder.followPathWithEvents(path);
//    }
}
