package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Aim extends CommandBase{
    private boolean isAligned = false;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");    

    public Aim() {

    }

    @Override
    public void execute() {
        NetworkTableEntry txEntry = table.getEntry("tx");

        double horizontalOffset = txEntry.getDouble(999);
        double zone = .2;

        //if the offest is more than the zone
        if(horizontalOffset > zone) {
            //strafe left
            RobotContainer.DRIVE.drive(0, 0, -.201, true);

        } else if (horizontalOffset < -zone) {
            RobotContainer.DRIVE.drive(0, 0, -.201, true);
        } else {
            isAligned = true;
            RobotContainer.DRIVE.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return isAligned;
    }
}