/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Mapping {
        public static class Drive {
            public static int frontLeft = 0;
            public static int backLeft = 1;
            public static int frontRight = 2;
            public static int backRight = 3;
        }
    }

    public static class Generic {
        public static int timeoutMs = 30;
    }

    public static class Config {
        public static class Input {
            public static double kInputDeadband = 0.2;
        }

        public static class Drive {
            public static class Kinematics {
                // Track Width (MUST BE IN METERS)
                public static double kTrackWidth = 0.6;

                public static final double kWheelDiameter = 8.4; // In Inches
                public static final double kInchesPerRotation = 26.5;
                public static final double kSensorUnitsPerRotation = 2048;
                public static final double kEncoderInchesPerCount = kWheelDiameter * Math.PI / kSensorUnitsPerRotation;
            }

            public static class Power {
                public static double kOpenLoopRamp = 0.2;

                public static StatorCurrentLimitConfiguration kStatorCurrentLimit = new StatorCurrentLimitConfiguration(
                        true, 30, 35, 100);
            }

            public static class GyroControl {
                public static double kP = 0.03;
                public static double kI = 0.0;
                public static double kD = 0.0;
                public static double kF = 0.0;

                public static double kToleranceDegrees = 1.0;
            }

            public static class MotionControl {
                public static int profileSlot = 0;

                public static class Left {
                    public static double kP = 0.0;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0;
                }

                public static class Right {
                    public static double kP = 0.0;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0;
                }
            }
        }
    }

    public static class DynConfig {
        public static class Drive {
            public static double VelocityDriveRPM = 250;
            public static double GyroTurnSpeed = 1;
        } 
    }
}
