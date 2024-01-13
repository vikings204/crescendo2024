// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.InterpLUT;

public final class Constants204 {
    public static final class DrivetrainCAN {
        public static final int FL_DRIVE_MOTOR_ID = 32;
        public static final int RL_DRIVE_MOTOR_ID = 35;
        public static final int FR_DRIVE_MOTOR_ID = 42;
        public static final int RR_DRIVE_MOTOR_ID = 45;

        public static final int FL_TURNING_MOTOR_ID = 33;
        public static final int RL_TURNING_MOTOR_ID = 34;
        public static final int FR_TURNING_MOTOR_ID = 43;
        public static final int RR_TURNING_MOTOR_ID = 44;

        public static final int SINGLE_STRAFE_DRIVE_MOTOR = 71;
        public static final int SINGLE_STRAFE_TURNING_MOTOR = 23;
    }

    public static final class ArmCAN {
        public static final int BOOM_MOTOR_ID = 11;
        public static final int DIPPER_MOTOR_ID = 21;
        public static final int /*CLAW_MOTOR_ID*/CLAW_SERVO_PWM_CH = 0; // controlled with PWM, NOT CAN
    }

    public static final class Arm {
        public static final double BOOM_REF_INCREMENT = 0.35;
        public static final double DIPPER_REF_INCREMENT = 0.45;
        public static final double CLAW_CLOSED_EXPOS = 0;
        public static final double CLAW_OPEN_EXPOS = 1;
    }

    public static final class Controller {
        public static final int PORT = 1;
        public static final double LX_DEADBAND = 0.1;
        public static final double LY_DEADBAND = 0.1;
        public static final double RX_DEADBAND = 0.1;
        public static final double RY_DEADBAND = 0.1;
        public static final double LX_MAG_DEADBAND = 0.1;
        public static final String PTZ_HOSTNAME = "http://10.2.4.69";
        public static final int PTZ_JOYSTICK_PORT = 2;
        
    }
    public static final class Drivetrain {
        public static final double FL_Input_1 = 0.0;
        public static final double FL_Input_2 = 90.0;
        public static final double FL_Input_3 = 180.0;
        public static final double FL_Input_4 = 270.0;
        public static final double FL_Output_1 = 0.0;
        public static final double FL_Output_2 = 90.0;
        public static final double FL_Output_3 = 180.0;
        public static final double FL_Output_4 = 270.0;
        public static final double RL_Input_1 = 0.0;
        public static final double RL_Input_2 = 90.0;
        public static final double RL_Input_3 = -90.0;
        public static final double RL_Input_4 = 180.0;
        public static final double RL_Output_1 = 0.0;
        public static final double RL_Output_2 = 90.0;
        public static final double RL_Output_3 = -90.0;
        public static final double RL_Output_4 = 180.0;
        public static final double FR_Input_1 = 0.0;
        public static final double FR_Input_2 = 90.0;
        public static final double FR_Input_3 = -90.0;
        public static final double FR_Input_4 = 180.0;
        public static final double FR_Output_1 = 0.0;
        public static final double FR_Output_2 = 90.0;
        public static final double FR_Output_3 = -90.0;
        public static final double FR_Output_4 = 180.0;
        public static final double RR_Input_1 = 0.0;
        public static final double RR_Input_2 = 90.0;
        public static final double RR_Input_3 = -90.0;
        public static final double RR_Input_4 = 180.0;
        public static final double RR_Output_1 = 0.0;
        public static final double RR_Output_2 = 90.0;
        public static final double RR_Output_3 = -90.0;
        public static final double RR_Output_4 = 180.0; 
        public static final InterpLUT FL_LUT = new InterpLUT();
        public static final InterpLUT RL_LUT = new InterpLUT();
        public static final InterpLUT FR_LUT = new InterpLUT();
        public static final InterpLUT RR_LUT = new InterpLUT();

        public static final double STRAFE_TURNING_PID_P = 5;
        public static final double STRAFE_TURNING_PID_I = 0.001;
        public static final double STRAFE_TURNING_PID_D = 0.000;
        public static final double STRAFE_DRIVE_PID_P = 0.1;
        public static double EQ_STRAFE_DIVISOR = 2; // speed limiter in EQ (AUGH)
        public static double EQ_ROTATE_DIVISOR = 6; // ^^^^^
        public static final double WHEEL_DIAMETER_M = 0.15;

        public static final double ROTATE_DRIVE_THRESHOLD = 25;
        public static final double STRAFE_DRIVE_THRESHOLD = 25;
    }

    public static final class Vision {
        public static final String PHOTONVISION_NAME = "OV9281";
        public static final double CAMERA_HEIGHT_METERS = 0.7112;
        public static final double TARGET_HEIGHT_METERS = 0.61;
        public static final double CAMERA_PITCH_DEGREES = 180;
    }

    public static final class Automation {
        public static final int DRIVE_BACKWARD_MS = 3000;
    }
}