package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Shoot extends CommandBase {

    public Shoot() {
    }

    @Override
    public void execute() {
        RobotContainer.SHOOTER.shoot();
    }
}