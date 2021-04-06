package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

    private DriveSubsystem drive = RobotContainer.DRIVE;

    public DriveCommand() {

    }

    @Override
    public void execute() {
        drive.drive(.4, 0, 0, true);
    }
}