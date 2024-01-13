// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.StrafeSubsystem;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.TagVisionSubsystem;
import frc.robot.util.*;
//import frc.robot.Auto.*;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import com.pathplanner.lib.util.*;
import com.pathplanner.lib.controllers.PathFollowingController;

/*import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;*/

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    //private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    //private final SingleStrafeSubsystem m_singleStrafeDrive = new SingleStrafeSubsystem();
   // public final StrafeSubsystem strafeDrive = new StrafeSubsystem();
    //public final ArmSubsystem armControl = new ArmSubsystem();
   // private final TagVisionSubsystem tagVision = new TagVisionSubsystem();
    public final Swerve s_Swerve = new Swerve();
    //public final generateTrajectory tracker = new generateTrajectory();

   // m_gyro.calibrate();
    //private final PTZCam ptzCam = new PTZCam();
    public int autoStateMachine = 0;
    Gamepad CONTROLLER = new Gamepad(Constants204.Controller.PORT);
    Joystick JOYSTICK = new Joystick(0);
   // private final CAN gyro = new CAN(1, 8, 4);
    //private final CANData gyroData = new CANData();
    private boolean balanceStop = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
  
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> CONTROLLER.getLeftX(),
                () -> -1*CONTROLLER.getLeftY(),
                () -> -1*CONTROLLER.getRightX(),
                () -> false,
                () -> CONTROLLER.getYButton(),
                () -> CONTROLLER.getAButton()));

                   
                if(CONTROLLER.getBButton()){
                    s_Swerve.m_gyro_P2.reset();
                   // s_Swerve.m_gyro_P2.calib;
                    System.out.println("you have calibed the gyro");
                }
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        

    }



    //public Command getAutonomousCommand() {}
       // return new boxandturn(this);}


    public double[] toEuler(double _x, double _y, double _z, double _w) {
        double[] ret = new double[3];
        double sqw = _w * _w;
        double sqx = _x * _x;
        double sqy = _y * _y;
        double sqz = _z * _z;

        ret[0] = Math.atan2(2.0 * (_x * _y + _z * _w), (sqx - sqy - sqz + sqw));
        ret[1] = Math.asin(-2.0 * (_x * _z - _y * _w) / (sqx + sqy + sqz + sqw));
        ret[2] = Math.atan2(2.0 * (_y * _z + _x * _w), (-sqx - sqy + sqz + sqw));

        return ret;
    }
}
