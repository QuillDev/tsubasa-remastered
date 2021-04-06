/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Aim;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static final DriveSubsystem DRIVE = new DriveSubsystem();
  public static final Intake INTAKE = new Intake();
  public static final Shooter SHOOTER = new Shooter();
  public static final Aim ALIGN = new Aim();
  public static final AutoCommand auto = new AutoCommand();

  // The driver's controller
  Joystick pilotStick = new Joystick(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Default command for the swerve subsystem, makes sure we drive when the driver
    // gives input
    DRIVE.setDefaultCommand(new RunCommand(
        () -> DRIVE.drive(-pilotStick.getRawAxis(1), pilotStick.getRawAxis(0), -pilotStick.getRawAxis(4), true),
        DRIVE));
    INTAKE.setDefaultCommand(new RunCommand(() -> INTAKE.drive(), INTAKE));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * calling passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Suppress unused warning for now
    // Configure controller buttons to be bound
    JoystickButton aButton = new JoystickButton(pilotStick, 1);
    JoystickButton bButton = new JoystickButton(pilotStick, 2);

    aButton.whileHeld(new RunCommand(() -> SHOOTER.shoot(), SHOOTER));
    aButton.whenInactive(new RunCommand(() -> SHOOTER.stop(), SHOOTER));

    bButton.whileHeld(ALIGN);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /**
     * // Create config for trajectory TrajectoryConfig config = new
     * TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
     * AutoConstants.kMaxAccelerationMetersPerSecondSquared) // Add kinematics to
     * ensure max speed is actually obeyed
     * .setKinematics(DriveConstants.kDriveKinematics);
     * 
     * // An example trajectory to follow. All units in meters. Trajectory
     * exampleTrajectory = TrajectoryGenerator.generateTrajectory( // Start at the
     * origin facing the +X direction new Pose2d(0, 0, new Rotation2d(0)), // Pass
     * through these two interior waypoints, making an 's' curve path List.of( new
     * Translation2d(1, 1), new Translation2d(2, -1) ), // End 3 meters straight
     * ahead of where we started, facing forward new Pose2d(3, 0, new
     * Rotation2d(0)), config );
     * 
     * SwerveControllerCommand swerveControllerCommand = new
     * SwerveControllerCommand( exampleTrajectory, swerveDrive::getPose,
     * //Functional interface to feed supplier DriveConstants.kDriveKinematics,
     * 
     * //Position controllers new PIDController(AutoConstants.kPXController, 0, 0),
     * new PIDController(AutoConstants.kPYController, 0, 0), new
     * ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
     * AutoConstants.kThetaControllerConstraints),
     * 
     * swerveDrive::setModuleStates,
     * 
     * swerveDrive
     * 
     * );
     * 
     * // Run path following command, then stop at the end. return
     * swerveControllerCommand.andThen(() -> swerveDrive.drive(0, 0, 0, false));
     */
    return auto;
  }
}
