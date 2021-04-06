package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

public class AutoCommand extends SequentialCommandGroup {

    private DriveCommand drive = new DriveCommand();
    private Shoot shoot = new Shoot();
    private Shooter shooter = RobotContainer.SHOOTER;
    private DriveSubsystem swerve = RobotContainer.DRIVE;

    public AutoCommand() {
        addCommands(shoot.withTimeout(8), new RunCommand(() -> shooter.stop(), shooter).withTimeout(1), drive.withTimeout(1.3), new RunCommand(() -> swerve.stop(), swerve) );
    }

    
}