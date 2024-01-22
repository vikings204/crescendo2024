package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.util.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final SwerveSubsystem SwerveSubsystem = new SwerveSubsystem();
    public final ShooterSubsystem ShooterSubsystem = new ShooterSubsystem();
    private final AutonomousManager AutonomousManager = new AutonomousManager(SwerveSubsystem);
    Gamepad CONTROLLER = new Gamepad(Constants204.Controller.PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();
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

        ShooterSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> ShooterSubsystem.speakerShot(CONTROLLER.getXButton()),
                        ShooterSubsystem));
    }
    private void configureButtonBindings() {
       // new JoystickButton(CONTROLLER, 5)
        //.whileTrue(
          //  new RunCommand(() -> ShooterSubsystem.receive(true), ShooterSubsystem));
    }

    public Command getAutonomousCommand() {
        return AutonomousManager.getCommand();
    }
}
