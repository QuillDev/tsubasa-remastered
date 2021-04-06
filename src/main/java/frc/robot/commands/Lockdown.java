package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;

public class Lockdown extends CommandBase {
    @SuppressWarnings({"unused"})
    private final DriveSubsystem driveSubsystem;

    /**
     * Lock down the robot making it hard to push by putting the wheels in an X shaped pattern
     */
    public Lockdown(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        //get the swerve modules
        SwerveModule[] modules = driveSubsystem.getModules();

        //set swerve modules to states that will form an X Shape
        modules[0].setDesiredState(new SwerveModuleState( 0, Rotation2d.fromDegrees(315) ) );
        modules[1].setDesiredState(new SwerveModuleState( 0, Rotation2d.fromDegrees(45)  ) );
        modules[2].setDesiredState(new SwerveModuleState( 0, Rotation2d.fromDegrees(135) ) );
        modules[3].setDesiredState(new SwerveModuleState( 0, Rotation2d.fromDegrees(225) ) );
        
        //add the drive subsystem as a requirement
        addRequirements(driveSubsystem);
    }
}