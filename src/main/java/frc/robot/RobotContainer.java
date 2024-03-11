package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controller;
import frc.robot.Robot.ControlMode;
import frc.robot.commands.ShootSpeakerCommand;
import frc.robot.commands.ShootSpeakerPoselessCommand;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.subsystems.*;
import frc.robot.util.Gamepad;

import static frc.robot.Robot.ControlModeChooser;
import static frc.robot.Robot.AutoModeChooser;

public class RobotContainer {
    public final SwerveSubsystem Swerve = new SwerveSubsystem();
    public final LEDSubsystem LED = new LEDSubsystem();
    public final ShooterSubsystem Shooter = new ShooterSubsystem(LED);
    public final LinearActuatorSubsystem LinearActuator = new LinearActuatorSubsystem();
    public final PoseEstimationSubsystem PoseEstimation = new PoseEstimationSubsystem(Swerve::getYaw, Swerve::getPositions);

    private final ShootSpeakerCommand ShootSpeakerCMD = new ShootSpeakerCommand(Swerve, PoseEstimation, Shooter);
    private final ShootSpeakerPoselessCommand ShootSpeakerPoselessCMD = new ShootSpeakerPoselessCommand(Swerve, PoseEstimation, Shooter);

    Gamepad DRIVER = new Gamepad(Controller.DRIVER_PORT);
    Gamepad OPERATOR = new Gamepad(Controller.OPERATOR_PORT);

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
        });

        AutoBuilder.configureHolonomic(
                PoseEstimation::getCurrentPose,
                PoseEstimation::setCurrentPose,
                Swerve::getSpeeds,
                Swerve::driveRobotRelative,
                Constants.Auto.PATH_FOLLOWER_CONFIG,
                () -> false,//Robot.alliance == DriverStation.Alliance.Red,
                Swerve
        );
        if (Robot.alliance == DriverStation.Alliance.Red) {
                PoseEstimation.disableEntry.setBoolean(true);
        }

        NamedCommands.registerCommand("intakeStart", new InstantCommand(() -> Shooter.receive(true), Shooter));
        NamedCommands.registerCommand("zeroGyro", new InstantCommand(Swerve::zeroGyro, Swerve));
        NamedCommands.registerCommand("intakeStop", new InstantCommand(() -> Shooter.receive(false), Shooter));
        NamedCommands.registerCommand("shooterStart", new InstantCommand(() -> Shooter.flywheelSpeaker(true), Shooter));
        NamedCommands.registerCommand("shooterStop", new InstantCommand(() -> Shooter.flywheelSpeaker(false), Shooter));
        NamedCommands.registerCommand("bumpStart", new InstantCommand(() -> Shooter.intake(true, false), Shooter));
        NamedCommands.registerCommand("bumpStop", new InstantCommand(() -> Shooter.intake(false, false), Shooter));
        NamedCommands.registerCommand("lowerIntake", new InstantCommand(() -> LinearActuator.shift(false,true), LinearActuator));
        NamedCommands.registerCommand("lowerIntakeStop", new InstantCommand(() -> LinearActuator.shift(false,false), LinearActuator));
        
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
                        () -> false,// DRIVER.getLeftStickButton(), // slow mode
                        () -> false));//DRIVER.getRightStickButton())); // fast mode

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
        new JoystickButton(DRIVER, 4)
                .whileTrue(
                        new RunCommand(Swerve::zeroGyro, Swerve));
        new JoystickButton(OPERATOR, 6)
                .whileTrue(
                        new RunCommand(() -> Shooter.intake(true, false), Shooter))
                .whileTrue(
                        new InstantCommand(() -> Shooter.detecting = true));
        // new JoystickButton(OPERATOR, 5)
        //         .whileTrue(
        //                 new RunCommand(() -> Shooter.receive(true), Shooter));
        new JoystickButton(OPERATOR, 4)
                .whileTrue(
                        new RunCommand(() -> Shooter.flywheelAmp(true), Shooter));

        new JoystickButton(OPERATOR, 1).whileTrue(ShootSpeakerCMD);
        new JoystickButton(OPERATOR, 2).whileTrue(ShootSpeakerPoselessCMD);
    }

    public Command getAutonomousCommand() {
        //Swerve.gyro.setYaw(-90.0); // temp for auto testing
        //return new PathPlannerAuto("Start Lower Second");
        return new PathPlannerAuto(AutoModeChooser.getSelected().pathplannerName);
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
