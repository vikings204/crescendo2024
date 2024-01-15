package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
    private SwerveSubsystem s_Swerve;
    private DoubleSupplier translationSup;    //+/- y direction
    private DoubleSupplier strafeSup;         // +/- x direction
    private DoubleSupplier rotationSup;       // spin CCW or CW
    private BooleanSupplier robotCentricSup;  // always false
    private BooleanSupplier slowSpeedSup;     //make robot go slow
    private BooleanSupplier highSpeedSup;     // make robot go fast - way too fast for now

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);  //Limits the rate of change of the voltage output to the motor to some maximum value.  Would change if you wanted the robot to "ramp" faster
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);     // See above
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);   //Not really used with the brushed motors going to a position but could be.  We don't use the trapazoid pid profile suggested only because it hasn't been implemented yet

    public TeleopSwerve(
            SwerveSubsystem s_Swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            BooleanSupplier slowSpeedSup,
            BooleanSupplier highSpeedSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);  //Adds the swerve subsystem as a req of the command and will not schedule any other commands that require the swerve sub


        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.slowSpeedSup = slowSpeedSup;
        this.highSpeedSup = highSpeedSup;
    }

    @Override
    public void execute() {
        double speedMultiplier = Constants.Swerve.normalDriveSpeedMultiplier;
        if (highSpeedSup.getAsBoolean()) speedMultiplier = Constants.Swerve.fastDriveSpeedMultiplier;
        if (slowSpeedSup.getAsBoolean()) speedMultiplier = Constants.Swerve.slowDriveSpeedMultiplier;

        /* Review the x,y and rotate inputs and run them through the rate limiter.  The code also applies a deadband
         * no input will be "Seen" by the robot unless the stick is passed the deadband.
         * Helps with not driving/turning by accident
         */
        double translationVal = translationLimiter.calculate(speedMultiplier
                                * MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband)
        );
        double strafeVal = strafeLimiter.calculate(speedMultiplier
                                * MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband)
        );
        double rotationVal = rotationLimiter.calculate(speedMultiplier
                                * MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband)
        );

        /* Drive
         * Constructs a new Translation2d vector object to tell the robot where to go to next in an x,y plane and a rotation double to
         * tell it how much to rotate
         */
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}
