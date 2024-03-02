package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot.ControlMode;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.util.Gamepad;

import static frc.robot.Constants.*;
import static frc.robot.Robot.ControlModeChooser;

public class RobotContainer {
    public final SwerveSubsystem Swerve = new SwerveSubsystem();
    public final LEDSubsystem LED = new LEDSubsystem();
    public final ShooterSubsystem Shooter = new ShooterSubsystem(LED.Presets::HasNote, LED.Presets::Default);
    public final LinearActuatorSubsystem LinearActuator = new LinearActuatorSubsystem();
    public final PoseEstimationSubsystem PoseEstimation = new PoseEstimationSubsystem(Swerve::getYaw, Swerve::getPositions);
    Gamepad DRIVER = new Gamepad(Controller.DRIVER_PORT);
    Gamepad OPERATOR = new Gamepad(Controller.DRIVER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        ControlModeChooser.onChange((ControlMode mode) -> {
            if (mode == ControlMode.SINGLE) {
                OPERATOR = new Gamepad(Controller.DRIVER_PORT);
            } else {
                OPERATOR = new Gamepad(Controller.OPERATOR_PORT);
            }
            configureDefaultCommands();
            configureButtonBindings();
            System.out.println("Switched control mode to " + mode);
        });

        NamedCommands.registerCommand("intakeStart", new InstantCommand(() -> Shooter.receive(true), Shooter));
        NamedCommands.registerCommand("intakeStop", new InstantCommand(() -> Shooter.receive(false), Shooter));
        NamedCommands.registerCommand("shooterStart", new InstantCommand(() -> Shooter.flywheelSpeaker(true), Shooter));
        NamedCommands.registerCommand("shooterStop", new InstantCommand(() -> Shooter.flywheelSpeaker(false), Shooter));
        NamedCommands.registerCommand("bumpStart", new InstantCommand(() -> Shooter.intake(true, false), Shooter));
        NamedCommands.registerCommand("bumpStop", new InstantCommand(() -> Shooter.intake(false, false), Shooter));


        configureDefaultCommands();
        configureButtonBindings();

        AutoBuilder.configureHolonomic(
                PoseEstimation::getCurrentPose,
                PoseEstimation::setCurrentPose,
                Swerve::getSpeeds,
                Swerve::driveRobotRelative,
                Constants.Auto.PATH_FOLLOWER_CONFIG,
                () -> Robot.alliance == DriverStation.Alliance.Red,
                Swerve
        );
    }

    private void configureDefaultCommands() {
        Swerve.setDefaultCommand(
                new TeleopSwerve(
                        Swerve,
                        () -> DRIVER.getLeftX(),
                        () -> -1 * DRIVER.getLeftY(),
                        () -> -1 * DRIVER.getRightX(),
                        () -> false,
                        () -> DRIVER.getYButton(),
                        () -> DRIVER.getAButton()));

        Shooter.setDefaultCommand(
                new RunCommand(
                        () -> Shooter.flywheelSpeaker(OPERATOR.getXButton()),
                        Shooter));

        LinearActuator.setDefaultCommand(
                new RunCommand(
                        () -> LinearActuator.shift(OPERATOR.getPOV() == 0, OPERATOR.getPOV() == 180),
                        LinearActuator
                )
        );
    }

    private void configureButtonBindings() {
        new JoystickButton(OPERATOR, 5)
                .whileTrue(
                        new RunCommand(() -> Shooter.intake(true, true), Shooter));
        new JoystickButton(OPERATOR, 6)
                .whileTrue(
                        new RunCommand(() -> Shooter.intake(true, false), Shooter));
        new JoystickButton(OPERATOR, 10)
                .whileTrue(
                        new RunCommand(() -> Shooter.flywheelAmp(true), Shooter));
    }

    public Command getAutonomousCommand() {
        Swerve.gyro.setYaw(-90.0); // temp for auto testing
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
