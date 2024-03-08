package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Map;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Constants.Vision.*;

public class ShootSpeakerCommand extends Command {
    private final SwerveSubsystem Swerve;
    private final PoseEstimationSubsystem PoseEst;
    private final ShooterSubsystem Shooter;
    private double initialTimestamp;

    private final ProfiledPIDController xPID = new ProfiledPIDController(3, 0, 0, new Constraints(Constants.Swerve.MAX_SPEED, 4));
    private final ProfiledPIDController yPID = new ProfiledPIDController(3, 0, 0, new Constraints(Constants.Swerve.MAX_SPEED, 4));
    private final ProfiledPIDController thetaPID = new ProfiledPIDController(2, 0, 0, new Constraints(Constants.Swerve.MAX_ANGULAR_VELOCITY, 4));
    private final GenericEntry secondsToShootEntry = Shuffleboard.getTab("SmartDashboard").add("seconds to shoot", (double) 1/3).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 1)).getEntry();


    public ShootSpeakerCommand(
            SwerveSubsystem Swerve,
            PoseEstimationSubsystem PoseEst,
            ShooterSubsystem Shooter
    ) {
        this.Swerve = Swerve;
        this.PoseEst = PoseEst;
        this.Shooter = Shooter;

        addRequirements(Swerve, Shooter);
    }

    @Override
    public void initialize() {
        initialTimestamp = getFPGATimestamp();
        Shooter.flywheelSpeaker(true);

        Translation2d destTranslation = Robot.alliance == DriverStation.Alliance.Blue ? SPEAKER_BLUE : SPEAKER_RED;
        Rotation2d destRotation = new Rotation2d(Robot.alliance == DriverStation.Alliance.Blue ? 0 : 1);

        xPID.setGoal(destTranslation.getX());
        yPID.setGoal(destTranslation.getY());
        thetaPID.setGoal(destRotation.getRadians());
    }
    @Override
    public void execute() {
        Pose2d robotPose = PoseEst.getCurrentPose();
//        Translation2d diffTrans = destTranslation.minus(robotPose.getTranslation());
//        Rotation2d diffRot = destRotation.minus(robotPose.getRotation());
//
//        if (getFPGATimestamp() > initialTimestamp+SECONDS_TO_SPIN_UP && diffTrans.getX() < 0.1 && diffTrans.getY() < 0.1 && diffRot.getDegrees() < 5) { // need to also check if close enough
//            Shooter.intake(true, false);
//        } else {
//            Swerve.drive(new Translation2d(xPID.calculate(robotPose.getX()), yPID.calculate(robotPose.getY())), thetaPID.calculate(robotPose.getRotation().getRadians()), true, false); // isOpenLoop differs from teleop
//        }

        Swerve.drive(new Translation2d(xPID.calculate(robotPose.getX()), yPID.calculate(robotPose.getY())), thetaPID.calculate(robotPose.getRotation().getRadians()), true, false); // isOpenLoop differs from teleop
        if (getFPGATimestamp() > initialTimestamp+secondsToShootEntry.getDouble((double) 1/3) && xPID.atGoal() && yPID.atGoal() && thetaPID.atGoal()) {
            Shooter.intake(true, false);
        }
    }
    @Override
    public void end(boolean interrupted) {
        Shooter.flywheelSpeaker(false);
        Shooter.intake(false, false);
    }
}
