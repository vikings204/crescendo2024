package frc.robot.util;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;

public class ReduceCANUsage {
    public static class SparkMax {
        // adapted from https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
        public enum Usage {
            kAll,
            kPositionOnly,
            kVelocityOnly,
            kMinimal
        }

        public static void setCANSparkMaxBusUsage(CANSparkMax motor, Usage usage/*, boolean enableFollowing*/) {
            if (/*enableFollowing*/DriverStation.isEStopped()) {
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
            } else {
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 500);
            }

            if (usage == Usage.kAll) {
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 50);
            } else if (usage == Usage.kPositionOnly) {
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
            } else if (usage == Usage.kVelocityOnly) {
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
            } else if (usage == Usage.kMinimal) {
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
                motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
            }
        }
    }
}
