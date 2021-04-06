package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private VictorSP intakeMotor = new VictorSP(0);

    public Intake() {
        System.out.println("Initialized Intake");
        intakeMotor.setInverted(true);
    }

    public void drive() {
        intakeMotor.set(-.45);
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public void unJam() {
        intakeMotor.set(-.4);
    }
}