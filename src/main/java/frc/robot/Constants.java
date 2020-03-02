/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

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
        public static int PCM_ID = 0;

        public static class Drive {
            public static int frontLeft = 0;
            public static int backLeft = 2;
            public static int frontRight = 1;
            public static int backRight = 3;
        }

        public static class Flywheel{
            public static int motor = 4;
        }

        public static class ControlPanel{
            public static int motor = 5;
        }

        public static class Intake{
            public static int motor = 6;
        }

        public static class IntakePneumatics{
            public static int solenoidLeftIn = 0;
            public static int solenoidLeftOut = 1;
            public static int solenoidRightIn = 2;
            public static int solenoidRightOut = 3;
        }

        public static class Hopper {
            public static int motor = 7;
        }

        public static class Climb {
            public static int master = 8;
            public static int slave1 = 9;
            public static int slave2 = 10;
            public static int slave3 = 11;
        }
    }

    public static class Generic {
        public static int timeoutMs = 100;
    }

    public static class Config {
        public static class Input {
            public static double kInputDeadband = 0.08;
        }

        public static class ColorSensor {
            public static Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
            public static Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
            public static Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
            public static Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
        }

        public static class ControlPanel {
            public static double kOpenLoopRamp = 0.0;
            public static double output = 0.25;
        }

        public static class Intake {
            public static double kOpenLoopRamp = 0.0;
        }

        public static class Flywheel{
            public static double kOpenLoopRamp = 0.0;

            public static class VelocityControl {
                public static int profileSlot = 0;
                public static final double kSensorUnitsPerRotation = 4096;
                
                public static double kP = 0.1;
                public static double kI = 0.0;
                public static double kD = 0.0;
                public static double kF = 0.0;
            }
        }
      
        public static class Hopper {
            public static double kOpenLoopRamp = 0.0;
            public static double output = 1.0;
        }

        public static class Drive {
            public static class Kinematics {
                // Track Width (MUST BE IN METERS)
                public static double kTrackWidth = 0.6;

                public static final double kWheelDiameter = 6; // In Inches
                public static final double kInchesPerRotation = kWheelDiameter * Math.PI;
                public static final double kSensorUnitsPerRotation = 2048; //2048 for talonfx
                public static final double kEncoderInchesPerCount = kWheelDiameter * Math.PI / kSensorUnitsPerRotation;

                public static final double kGearRatio = (12.0 * 20.0) / (50.0 * 54.0);
            }

            public static class Power {
                public static double kOpenLoopRamp = 0.0;
                public static double kClosedLoopRamp = 0.0;

                public static StatorCurrentLimitConfiguration kStatorCurrentLimit = new StatorCurrentLimitConfiguration(
                        true, 30, 35, 100);
            }

            public static class GyroControl {
                public static double kP = 0.006;
                public static double kI = 0.01;
                public static double kD = 0.0;

                public static double kToleranceDegrees = 1.0;
            }

            public static class VelocityControl {
                public static int profileSlot = 0;

                public static class Left {
                    public static double kP = 0.1;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0468;
                }

                public static class Right {
                    public static double kP = 0.1;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0468;
                }
            }

            public static class MotionMagic {
                public static int profileSlot = 1;

                public static int maxVel = 10000;
                public static int maxAcc = 5000;

                public static class Left {
                    public static double kP = 0.3;
                    public static double kI = 0.0;
                    public static double kD = 0.01;
                    public static double kF = 0.0;
                }

                public static class Right {
                    public static double kP = 0.3;
                    public static double kI = 0.0;
                    public static double kD = 0.01;
                    public static double kF = 0.0;
                }
            }

            public static class MotionProfile {
                public static int profileSlot = 2;

                public static double kToleranceDegrees = 1.0;

                public static class Left {
                    public static double kP = 0.5;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0;
                }

                public static class Right {
                    public static double kP = 0.5;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0;
                }
            }
        }
    }

    public static class DynConfig {
        public static class Drive {
            public static double VelocityDriveRPM = 4000;
            public static double GyroTurnSpeed = 4;
        }

        public static class Flywheel{
            public static double RPM = 3000;
        }
    }
}
