/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 2;
    public static final int kbackLeftDriveMotorPort = 8;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kbackRightDriveMotorPort = 1;
    public static final double kFontLeftOffset = 0;
    public static final double kFontRightOffset = 0;
    public static final double kBackLeftOffset = 0;
    public static final double kBackRightOffset = 0;

    public static final int kFrontLeftTurningMotorPort = 7;
    public static final int kbackLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 6;
    public static final int kbackRightTurningMotorPort = 4;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kbackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kbackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kbackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kbackRightDriveEncoderReversed = false;

    public static final boolean kFrontLeftSteerInversed = false;
    public static final boolean kFrontRightSteerInversed = false;
    public static final boolean kBackLeftSteerInversed = false;
    public static final boolean kBackRightSteerInversed = false;

    public static final double kTrackWidth = 0.4826;
    //Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.5334;
    //Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3.5;
    public static final double speedLimit = .6;
  }

  public static final class ModuleConstants {

    //Drive encoder data configuration
    public static final double gearRatio = 6.67;
    public static final int driveEncoderCPR = 42; // Counts per revolution of the neo
    public static final double wheelDiameterMetres = 0.1016; // Wheel diameter in metres of the wheel
    public static final double distancePerPulse = (wheelDiameterMetres * Math.PI) / (double) driveEncoderCPR; //driven distance per pulse
    public static final double driveVelocityConversion =  ( (wheelDiameterMetres * Math.PI) / 60 ); // RPM to MPS Ratio
    public static final double speedLimit = .6;

    //Steering encoder data configuration
    public static final int kEncoderCPR = 4096;
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double maxModuleAngularSpeedEncUnitsPerSecond = kEncoderCPR;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
    public static final double maxModuleAngularAccelerationRadiansPerSecondSquared = kEncoderCPR;
    public static final double kTurningEncoderDistancePerPulse = (2 * Math.PI) / (double) kEncoderCPR;

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    //Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
          kMaxAngularSpeedRadiansPerSecondSquared);

  }
}