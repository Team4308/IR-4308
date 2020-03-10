/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import bbb.path.Gains;
import bbb.path.PathFollowerSettings;
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

        public static class Flywheel {
            public static int motor = 4;
        }

        public static class ControlPanel {
            public static int motor = 5;
        }

        public static class Intake {
            public static int intakeMotor = 6;
            public static int conveyorMotor = 7;
            public static int hopperMotor = 8;
            public static int topRoller = 9;
        }

        public static class Climb {
            public static int master = 10;
            public static int slave1 = 11;
            public static int slave2 = 12;
            public static int slave3 = 13;
        }

        public static class ClimbArm {
            public static int motor = 14;
        }
    }

    public static class Generic {
        public static int timeoutMs = 100;
    }

    public static class Config {
        public static class Input {
            public static double kInputDeadband = 0.14;

            public static class DriveStick {
                public static double kInputPrecision = 0.6;
                public static double kInputScale = 3.0;
            }
        }

        public static class ControlPanel {
            public static double kOpenLoopRamp = 0.0;
        }

        public static class Intake {
            public static double kOpenLoopRamp = 0.0;
        }

        public static class Flywheel {
            public static double kOpenLoopRamp = 0.0;

            public static class VelocityControl {
                public static int profileSlot = 0;
                public static final double kSensorUnitsPerRotation = 4096;

                public static double kP = 0.16;
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
                public static final double kSensorUnitsPerRotation = 2048; // 2048 for talonfx
                public static final double kEncoderInchesPerCount = kWheelDiameter * Math.PI / kSensorUnitsPerRotation;

                public static final double kGearRatio = (12.0 * 20.0) / (50.0 * 54.0);
            }

            public static class Power {
                public static double kOpenLoopRamp = 0.0;
                public static double kClosedLoopRamp = 0.0;

                public static StatorCurrentLimitConfiguration kStatorCurrentLimit = new StatorCurrentLimitConfiguration(
                        true, 35, 40, 100);
            }

            public static class GyroControl {
                public static double kP = 0.012;
                public static double kI = 0.0002;
                public static double kD = 0.0;

                public static double kToleranceDegrees = 1.0;
            }

            public static class VelocityControl {
                public static int profileSlot = 0;

                public static class Left {
                    public static double kP = 0.2;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0468;
                }

                public static class Right {
                    public static double kP = 0.2;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0468;
                }
            }

            public static class MotionMagic {
                public static int profileSlot = 1;

                public static int maxVel = 15000;
                public static int maxAcc = 6000;

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
                public static int period = 10; // In milliseconds

                public static class Left {
                    public static double kP = 0.01;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0;
                    public static double kV = 0.0;
                    public static double ka = 0.0;
                }

                public static Gains leftGains = new Gains(Left.kP, Left.kI, Left.kD, Left.kF, Left.kV, Left.ka);

                public static class Right {
                    public static double kP = 0.01;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0;
                    public static double kV = 0.0;
                    public static double ka = 0.0;
                }

                public static Gains rightGains = new Gains(Right.kP, Right.kI, Right.kD, Right.kF, Right.kV, Right.ka);

                public static class Turn {
                    public static double kP = 0.01;
                    public static double kI = 0.0;
                    public static double kD = 0.0;
                    public static double kF = 0.0;
                    public static double kV = 0.01;
                    public static double ka = 0.01;

                    public static double kToleranceDegrees = 1.0;
                }

                public static Gains turnGains = new Gains(Turn.kP, Turn.kI, Turn.kD, Turn.kF, Turn.kV, Turn.ka,
                        Turn.kToleranceDegrees);

                public static PathFollowerSettings settings = new PathFollowerSettings(
                        Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation,
                        Constants.Config.Drive.Kinematics.kGearRatio, Constants.Config.Drive.MotionProfile.leftGains,
                        Constants.Config.Drive.MotionProfile.rightGains, Constants.Config.Drive.MotionProfile.turnGains,
                        Constants.Config.Drive.VelocityControl.profileSlot,
                        Constants.Config.Drive.MotionProfile.period);
            }
        }
    }

    public static class DynConfig {
        public static class Drive {
            public static double VelocityDriveRPM = 5000;
            public static double GyroTurnSpeed = 4;
        }

        public static class Flywheel {
            public static double RPM = 3000;
        }
    }
}
