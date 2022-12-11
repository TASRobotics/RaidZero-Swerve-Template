// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Swerve constants
    public static final class SwerveConstants {

        // Rotor IDs
        public static final int kLeftFrontRotorID = 1;
        public static final int kRightFrontRotorID = 2;
        public static final int kLeftRearRotorID = 3;
        public static final int kRightRearRotorID = 4;

        // Throttle IDs
        public static final int kLeftFrontThrottleID = 5;
        public static final int kRightFrontThrottleID = 6;
        public static final int kLeftRearThrottleID = 7;
        public static final int kRightRearThrottleID = 8;

        // Rotor encoder IDs
        public static final int kLeftFrontCANCoderID = 9;
        public static final int kRightFrontCANCoderID = 10;
        public static final int kLeftRearCANCoderID = 11;
        public static final int kRightRearCANCoderID = 12;

        // Rotor encoder & motor inversion
        public static final boolean kRotorEncoderDirection = false;
        public static final boolean kRotorMotorInversion = false;

        // IMU ID
        public static final int kImuID = 0;

        // Rotor encoder offsets
        public static final double kLeftFrontRotorOffset = 0.0;
        public static final double kRightFrontRotorOffset = 0.0;
        public static final double kLeftRearRotorOffset = 0.0;
        public static final double kRightRearRotorOffset = 0.0;

        // Swerve kinematics; see documentation for more details
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.0, 0.0), 
            new Translation2d(0.0, 0.0), 
            new Translation2d(0.0, 0.0),
            new Translation2d(0.0, 0.0)
        );

        // Rotor PID constants
        public static final double kRotor_kP = 0.0;
        public static final double kRotor_kI = 0.0;
        public static final double kRotor_kD = 0.0;

        // Velocity & acceleration of swerve
        public static final double kMaxVelocityMetersPerSecond = 3.0;
        public static final double kMaxAccelerationMetersPerSecond = 3.0;

        // Wheel diameter
        public static final double kWheelDiameterMeters = 0.0; // wheel diameter
        
        // Throttle gear ratio
        // (number of turns it takes the motor to rotate the rotor one revolution)
        public static final double kThrottleGearRatio = 0.0; 

        // This value will be multiplied to the raw RPM of the throttle motor
        // and should convert it to meters per second
        // This is the general formula: 
        //     (1.0 / GEAR RATIO / 60.0_seconds) * WHEEL DIAMETER * Math.PI;
        public static final double kThrottleVelocityConversionFactor = 
            (1/kThrottleGearRatio/60)*kWheelDiameterMeters*Math.PI;
        
        // Pathing PID constants 
        public static final double kPathingX_kP = 0.1;
        public static final double kPathingX_kI = 0.0;
        public static final double kPathingX_kD = 0.0;

        public static final double kPathingY_kP = 0.1;
        public static final double kPathingY_kI = 0.0;
        public static final double kPathingY_kD = 0.0;

        public static final double kPathingTheta_kP = 0.1;
        public static final double kPathingTheta_kI = 0.0;
        public static final double kPathingTheta_kD = 0.0;
    }
    // Voltage compensation
    public static final double kVoltageCompensation = 12.0;
    
    public static final int kControllerPort = 0;
}
