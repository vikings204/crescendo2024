package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Map;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

public class TimedSpeakerShotCommand extends Command {
    private final ShooterSubsystem Shooter;
    private double initialTimestamp;
    private final GenericEntry secondsToShootEntry = Shuffleboard.getTab("config").add("seconds to shoot p", (double) 1).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 1)).getEntry();

    public TimedSpeakerShotCommand(
            ShooterSubsystem Shooter
    ) {
        this.Shooter = Shooter;

        addRequirements(Shooter);
    }

    @Override
    public void initialize() {
        initialTimestamp = getFPGATimestamp();
        Shooter.flywheelSpeaker(true);
    }
    @Override
    public void execute() {
        if (getFPGATimestamp() > initialTimestamp + secondsToShootEntry.getDouble((double) 1/3)) {
            Shooter.intake(true, false);
        }
    }
    @Override
    public void end(boolean interrupted) {
        Shooter.flywheelSpeaker(false);
        Shooter.intake(false, false);
    }
}
