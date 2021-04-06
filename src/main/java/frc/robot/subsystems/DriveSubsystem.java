/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
    // Robot swerve modules'
    private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed, DriveConstants.kFrontLeftSteerInversed, "Front Left");

    private final SwerveModule m_backLeft = new SwerveModule(DriveConstants.kbackLeftDriveMotorPort,
            DriveConstants.kbackLeftTurningMotorPort, DriveConstants.kbackLeftDriveEncoderReversed,
            DriveConstants.kbackLeftTurningEncoderReversed, DriveConstants.kBackLeftSteerInversed, "Back Left");

    private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed, DriveConstants.kFrontRightSteerInversed, "Front Right");

    private final SwerveModule m_backRight = new SwerveModule(DriveConstants.kbackRightDriveMotorPort,
            DriveConstants.kbackRightTurningMotorPort, DriveConstants.kbackRightDriveEncoderReversed,
            DriveConstants.kbackRightTurningEncoderReversed, DriveConstants.kBackRightSteerInversed, "Back Right");

    // The navX sensor
    private final AHRS navX = new AHRS();
    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getAngle());

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        reset();

        m_frontLeft.resetEncoders();
        m_backLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_backRight.resetEncoders();

    }

    /**
     * Returns the angle of the robot as a Rotation2d.
     *
     * @return The angle of the robot.
     */
    public Rotation2d getAngle() {
        // Negating the angle because WPILib gyros are CW positive.
        return Rotation2d.fromDegrees(-Math.IEEEremainder(navX.getAngle(), 360));
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(new Rotation2d(getHeading()), m_frontLeft.getState(), m_backLeft.getState(),
                m_frontRight.getState(), m_backRight.getState());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, getAngle());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    double deadzone = .2;

    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        if (Math.abs(xSpeed) < deadzone) {
            xSpeed = 0;
        }

        if (Math.abs(ySpeed) < deadzone) {
            ySpeed = 0;
        }

        if (Math.abs(rot) < deadzone) {
            rot = 0;
        }

        ySpeed *= -1;
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_backLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_backRight.resetEncoders();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        navX.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
        return navX.getYaw();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return navX.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Stop motors
     */
    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    /**
     * Reset the encoders on each module
     */
    public void reset() {
        m_frontLeft.reset();
        m_frontRight.reset();
        m_backLeft.reset();
        m_backRight.reset();
    }

    /**
     * Get the swerve modules and return them as an array
     * 
     * @return array of the swerve modules
     */
    public SwerveModule[] getModules() {

        // Construct array of the swerve modules
        SwerveModule[] modules = { m_frontLeft, m_frontRight, m_backLeft, m_backRight };

        // return the modules
        return modules;
    }

}
