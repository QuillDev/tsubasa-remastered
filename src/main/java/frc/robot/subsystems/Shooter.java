package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private CANSparkMax shooter = new CANSparkMax(15, MotorType.kBrushless);
    private CANEncoder encoder = shooter.getEncoder();
    private CANSparkMax indexer = new CANSparkMax(16, MotorType.kBrushless);
    private CANPIDController shooterPID = shooter.getPIDController();

    public Shooter() {
        shooter.restoreFactoryDefaults();
        indexer.restoreFactoryDefaults();

        // PID coefficients
        double kP = 2;
        double kI = .003;
        double kD = 0;
        double kIz = 0;
        double kFF = 0.000015;
        double kMaxOutput = 1;
        double kMinOutput = -1;

        // set PID coefficients
        shooterPID.setP(kP);
        shooterPID.setI(kI);
        shooterPID.setD(kD);
        shooterPID.setIZone(kIz);
        shooterPID.setFF(kFF);
        shooterPID.setOutputRange(kMinOutput, kMaxOutput);
    }

    /**
     * Start up the shooter
     */
    public void shoot() {
        // set the shooter to max speedss
        shooter.set(1);
        SmartDashboard.putNumber("Shooter RPM", encoder.getVelocity());
        // check if the shooter is up to RPM and if the target is in sight
        if (Math.abs(encoder.getVelocity()) >= 5700) {
            indexer.set(1);
        } else {
            indexer.set(-.01);
        }
    }

    public void stop() {
        shooter.set(0);
        indexer.set(0);

    }

}