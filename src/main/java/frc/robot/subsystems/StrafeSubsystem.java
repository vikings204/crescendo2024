// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants204;
import frc.robot.util.Math204;
import frc.robot.util.PolarCoordinate;
//lklkl

public class StrafeSubsystem extends SubsystemBase {
    // FL-frontleft, RL-rearleft. FR-frontright, RR-rearright
    public final StrafeModule FL_MOD =
            new StrafeModule(
                    Constants204.DrivetrainCAN.FL_DRIVE_MOTOR_ID,
                    Constants204.DrivetrainCAN.FL_TURNING_MOTOR_ID
            );

    public final StrafeModule RL_MOD =
            new StrafeModule(
                    Constants204.DrivetrainCAN.RL_DRIVE_MOTOR_ID,
                    Constants204.DrivetrainCAN.RL_TURNING_MOTOR_ID
            );

    public final StrafeModule FR_MOD =
            new StrafeModule(
                    Constants204.DrivetrainCAN.FR_DRIVE_MOTOR_ID,
                    Constants204.DrivetrainCAN.FR_TURNING_MOTOR_ID
            );

    public final StrafeModule RR_MOD =
            new StrafeModule(
                    Constants204.DrivetrainCAN.RR_DRIVE_MOTOR_ID,
                    Constants204.DrivetrainCAN.RR_TURNING_MOTOR_ID
            );
    private double turningPDeg = 0;
    private int turningPQuad = 0; // top left is 1 counterclockwise
    public double turningTotalDeg = 0.0;

    public StrafeSubsystem() {
    }

    @Override
    public void periodic() {
        //System.out.println("Target Degrees "+turningTotalDeg); 
    }

    public void basicDrive(double forward, double strafe, double rot) {
        //System.out.println("FW:"+forward + " ST:"+strafe + " RT:"+rot);
        double r = EQ.rotate(rot);
        double s = EQ.strafe(strafe);
        double f = EQ.forward(forward);
        //System.out.println("F:"+f+" S:"+s+" R:"+r);
        if (r != 0) {
            // FL-135 FR-45 RL-225 RR-315
            FL_MOD.rotate(135, r);
            FR_MOD.rotate(45, r);
            RL_MOD.rotate(-135, r);
            RR_MOD.rotate(-45, r);
        } else if (s != 0) {
            FL_MOD.strafe(s);
            FR_MOD.strafe(s);
            RL_MOD.strafe(s);
            RR_MOD.strafe(s);
        } else if (f != 0) {
            FL_MOD.forward(f);
            FR_MOD.forward(f);
            RL_MOD.forward(f);
            RR_MOD.forward(f);
        } else {
            FL_MOD.resetPos(0.0);
            FR_MOD.resetPos(0.0);
            RL_MOD.resetPos(0.0);
            RR_MOD.resetPos(0.0);
        }
    }

    public void moreDrive(double sx, double sy, double rot) {
        double r = EQ.rotate(rot);
        PolarCoordinate pc = Math204.CartesianToPolar(sy*-1, sx*-1);
        pc.mag = EQ.strafeMag(pc.mag*-1);
        if (r != 0) {
            // FL-135 FR-45 RL-225 RR-315
           /* m_frontLeft.rotate((int)Constants204.Drivetrain.FL_LUT.get(135.0), r);
            m_frontRight.rotate((int)Constants204.Drivetrain.FR_LUT.get(45.0), r);
            m_rearLeft.rotate((int)Constants204.Drivetrain.RL_LUT.get(-135.0), r);
            m_rearRight.rotate((int)Constants204.Drivetrain.RR_LUT.get(-45.0), r);*/
                        FL_MOD.rotate(135, r);
                       FR_MOD.rotate(45, r);
                       RL_MOD.rotate(-135, r);
                       RR_MOD.rotate(-45, r);
        } else if (pc.mag != 0) {
            //System.out.println("SX:" + sx + " SY:" + sy);
           //System.out.println("MAG:" + pc.mag);
            pc.deg = SwerveContinuous(pc.deg);
           // System.out.println("CDEG: " + pc.deg);
           FL_MOD.fullStrafe(pc);
            FR_MOD.fullStrafe(pc);
            RL_MOD.fullStrafe(pc);
            RR_MOD.fullStrafe(pc);
            
        } else {
           /*turningTotalDeg= resetTotal(turningTotalDeg) ;
           if( turningTotalDeg%360 == 0){ 
           m_frontLeft.resetPos(turningTotalDeg);
            m_frontRight.resetPos(turningTotalDeg);
            m_rearLeft.resetPos(turningTotalDeg);
            m_rearRight.resetPos(turningTotalDeg);
            System.out.println("IM HERERERERERE TTD:" + turningTotalDeg );
           }*/
           //m_frontLeft.setZero();
            //m_frontRight.setZero();
            //m_rearLeft.setZero();
            //m_rearRight.setZero();*/
            FL_MOD.resetPos(0);
            FR_MOD.resetPos(0);
            RL_MOD.resetPos(0);
            RR_MOD.resetPos(0);
            turningTotalDeg = 0.0;
            turningPDeg = 0.0;
            turningPQuad = 1;
            System.out.println("Front Left Enc: "+FL_MOD.turningMotor.getSelectedSensorPosition()+"\nFront Right Enc: "+FR_MOD.turningMotor.getSelectedSensorPosition()+"\nBack Left Enc: "+RL_MOD.turningMotor.getSelectedSensorPosition()+"Back Right Enc: "+RR_MOD.turningMotor.getSelectedSensorPosition());
            //FL_MOD.setBrakeMode();
            //FR_MOD.setBrakeMode();
            //RL_MOD.setBrakeMode();
            //RR_MOD.setBrakeMode();
        }
        //System.out.println("Target Degrees "+turningTotalDeg); 
    }

    public void polarDrive(PolarCoordinate pc, double rot) {
        double r = EQ.rotate(rot);
        pc.mag = EQ.strafeMag(pc.mag);
        if (r != 0) {
            // FL-135 FR-45 RL-225 RR-315
            FL_MOD.rotate(135, r);
            FR_MOD.rotate(45, r);
            RL_MOD.rotate(-135, r);
            RR_MOD.rotate(-45, r);
        } else if (pc.mag != 0) {
            pc.deg = SwerveContinuous(pc.deg);
            FL_MOD.fullStrafe(pc);
            FR_MOD.fullStrafe(pc);
            RL_MOD.fullStrafe(pc);
            RR_MOD.fullStrafe(pc);
        } else {
            FL_MOD.resetPos(0);
            FR_MOD.resetPos(0);
            RL_MOD.resetPos(0);
            RR_MOD.resetPos(0);
            turningTotalDeg = 0.0;
            turningPDeg = 0.0;
            turningPQuad = 1;
        }
    }

    public void balanceStop() {
        FL_MOD.turn(45);
        FR_MOD.turn(135);
        RL_MOD.turn(-45);
        RR_MOD.turn(-135);
    }
 
    public void setZero() {
        FL_MOD.setZero();
        FR_MOD.setZero();
        RL_MOD.setZero();
        RR_MOD.setZero();
        turningTotalDeg = 0.0;
    }

    public void rottenest() {
       /*m_frontLeft.rotate(0, 0.1);
        m_frontLeft.rotate(90, 0.1);
        m_frontLeft.rotate(180, 0.1);
        m_frontLeft.rotate(270, 0.1);
        m_frontLeft.rotate(360, 0.1);
        m_frontLeft.rotate(450, 0.1);
        m_frontLeft.rotate(540, 0.1);
        m_frontLeft.rotate(630, 0.1);
        m_frontLeft.rotate(720, 0.1);*/
    }

    private double SwerveContinuous(double cDeg) {
        double nDeg;
        int cQuad = Math204.GetQuadrant(cDeg);
        if ((turningPQuad == 3 || turningPQuad == 4) && cQuad == 1) {
            nDeg = (360-turningPDeg)+cDeg;
        } else if ((turningPQuad == 1 || turningPQuad == 2) && cQuad == 4) {
            nDeg = -((360-cDeg)+turningPDeg);
        } else {
            nDeg = cDeg - turningPDeg;
        }

        turningTotalDeg += nDeg;
        turningPDeg = cDeg;
        turningPQuad = cQuad;
        return turningTotalDeg;
    }

    private static class EQ {
        public static double forward(double in) {
            if (Math.abs(in) < Constants204.Controller.LY_DEADBAND) {
                return 0.0;
            } else {
                return in/2;
            }
        }

        public static double strafe(double in) {
            if (Math.abs(in) < Constants204.Controller.LX_DEADBAND) {
                return 0.0;
            } else {
                return in/3;
            }
        }

        public static double rotate(double in) {
            if (Math.abs(in) < Constants204.Controller.RX_DEADBAND) {
                return 0.0;
            } else {
                return in/Constants204.Drivetrain.EQ_ROTATE_DIVISOR;
            }
        }

        public static double strafeMag(double in) {
            if (Math.abs(in) < Constants204.Controller.LX_MAG_DEADBAND) {
                return 0.0;
            } else {
                return in/Constants204.Drivetrain.EQ_STRAFE_DIVISOR;
            }
        }
    }

    public String TestEncoders() {
        return ("FL DEG: " + FL_MOD.getTurnEncDeg() +
        "\nFR DEG: " + FR_MOD.getTurnEncDeg() +
        "\nRL DEG: " + RL_MOD.getTurnEncDeg() +
        "\nRR DEG: " + RR_MOD.getTurnEncDeg() +
        "\n==========================");
    }
    public double resetTotal(double total) {
        int revolutions = ((int)total)/360;
       // System.out.println("Target Degrees "+unitConv(360*revolutions));
        //turningMotor.set(TalonSRXControlMode.Position, unitConv(360*revolutions));
        if (revolutions != 0){
        System.out.println("Revolutions: "+ revolutions + " Degree Target: " + (double)revolutions*360.0);
       // driveMotor.set(0);
        }
        return 360.0* (double)revolutions;
    }
    }

