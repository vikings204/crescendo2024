package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot.AutoMode;
import frc.robot.Robot.ControlMode;
import frc.robot.commands.ShootSpeakerCommand;
import frc.robot.commands.ShootSpeakerPoselessCommand;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.subsystems.*;
import frc.robot.util.Gamepad;

import static frc.robot.Constants.*;
import static frc.robot.Robot.ControlModeChooser;
import static frc.robot.Robot.AutoModeChooser;

public class RobotContainer {
    public final SwerveSubsystem Swerve = new SwerveSubsystem();
    public final LEDSubsystem LED = new LEDSubsystem();
    public final ShooterSubsystem Shooter = new ShooterSubsystem(LED.Presets::HasNote, LED.Presets::Default);
    public final LinearActuatorSubsystem LinearActuator = new LinearActuatorSubsystem();
    public final PoseEstimationSubsystem PoseEstimation = new PoseEstimationSubsystem(Swerve::getYaw, Swerve::getPositions);
    Gamepad DRIVER = new Gamepad(Controller.DRIVER_PORT);
    Gamepad OPERATOR = new Gamepad(Controller.DRIVER_PORT);
    PathPlannerAuto pathChosen ;//= new PathPlannerAuto(null);

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
        AutoBuilder.configureHolonomic(
                PoseEstimation::getCurrentPose,
                PoseEstimation::setCurrentPose,
                Swerve::getSpeeds,
                Swerve::driveRobotRelative,
                Constants.Auto.PATH_FOLLOWER_CONFIG,
                () -> Robot.alliance == DriverStation.Alliance.Red,
                Swerve
        );
        AutoModeChooser.onChange((AutoMode mode1) -> {
            if (mode1 == AutoMode.MidToTop) {
                pathChosen = new PathPlannerAuto("Top Note");
            } 
            else  if (mode1 == AutoMode.MidToBot) {
                pathChosen = new PathPlannerAuto("Bottom Note");
            } 
            else  if (mode1 == AutoMode.ToptoTop) {
                pathChosen = new PathPlannerAuto("Top to Top Note");
            } 
            else   if (mode1 == AutoMode.BottoBot) {
                pathChosen = new PathPlannerAuto("Copy of Bottom Side to Bottom");
            } 
            else {
                pathChosen = new PathPlannerAuto("Top Note");
            }
        
           // System.out.println("Switched control mode to " + mode);
        });

        NamedCommands.registerCommand("intakeStart", new InstantCommand(() -> Shooter.receive(true), Shooter));
        NamedCommands.registerCommand("zeroGyro", new InstantCommand(Swerve::zeroGyro, Swerve));
        NamedCommands.registerCommand("intakeStop", new InstantCommand(() -> Shooter.receive(false), Shooter));
        NamedCommands.registerCommand("shooterStart", new InstantCommand(() -> Shooter.flywheelSpeaker(true), Shooter));
        NamedCommands.registerCommand("shooterStop", new InstantCommand(() -> Shooter.flywheelSpeaker(false), Shooter));
        NamedCommands.registerCommand("bumpStart", new InstantCommand(() -> Shooter.intake(true, false), Shooter));
        NamedCommands.registerCommand("bumpStop", new InstantCommand(() -> Shooter.intake(false, false), Shooter));
        NamedCommands.registerCommand("lowerIntake", new InstantCommand(() -> LinearActuator.shift(false,true), LinearActuator));

        configureDefaultCommands();
        configureButtonBindings();


    }

    private void configureDefaultCommands() {
        Swerve.setDefaultCommand(
                new TeleopSwerveCommand(
                        Swerve,
                        () -> -1*DRIVER.getLeftX(),
                        () ->   DRIVER.getLeftY(),
                        () -> -1 * DRIVER.getRightX(),
                        () -> false,
                        () -> DRIVER.getLeftStickButton(), // slow mode
                        () -> DRIVER.getYButton())); // fast mode

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
        new JoystickButton(OPERATOR, 4)
                .whileTrue(
                        new RunCommand(Swerve::zeroGyro, Swerve));
        new JoystickButton(OPERATOR, 6)
                .whileTrue(
                        new RunCommand(() -> Shooter.intake(true, false), Shooter));
        new JoystickButton(OPERATOR, 10)
                .whileTrue(
                        new RunCommand(() -> Shooter.flywheelAmp(true), Shooter));
        new JoystickButton(OPERATOR, 1)
                .whileTrue(new ShootSpeakerCommand(Swerve, PoseEstimation, Shooter));
        new JoystickButton(OPERATOR, 2)
                .whileTrue(new ShootSpeakerPoselessCommand(Swerve, PoseEstimation, Shooter));
    }

    public Command getAutonomousCommand() {
        //Swerve.gyro.setYaw(-90.0); // temp for auto testing
        //return new PathPlannerAuto("Start Lower Second");
        return pathChosen;
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
