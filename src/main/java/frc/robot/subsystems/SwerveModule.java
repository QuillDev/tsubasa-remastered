/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  //Create motor controllers
  private final CANSparkMax drive;
  private final TalonSRX steer;

  //create encoders
  private final CANEncoder driveEncoder;
  private final SensorCollection steerEncoder;
  private final boolean turningEncoderReversed;
  private final boolean turningMotorInverse;

  private final int kTimeoutMs = 10;
  
  //create PIDController for the drive velocity
  private final CANPIDController drivePID;


  private String name;
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, boolean driveEncoderReversed, boolean turningEncoderReversed,boolean inverseTurn, String name) {

    drive = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    steer = new TalonSRX(turningMotorChannel);

    this.turningMotorInverse = inverseTurn;

    this.turningEncoderReversed = turningEncoderReversed;
    this.name = name;

    //get the pid controller from the motor
    drivePID = drive.getPIDController();

    //get the pid controller from the encoder
    driveEncoder = drive.getEncoder();

    //get the steer encoder
    steerEncoder = steer.getSensorCollection();

    //setup the steer encoder
    setupSteerEncoder();

    //setup the drive motor
    setupDriveEncoder();
  }

  private void setupSteerEncoder() {
    steer.configFactoryDefault();
    //sent command timeout so we dont get locked on a command
    int kTimeoutMs = 10;

    //PID slot to use
    int kPIDLoopIdx = 0;

    //Ramp and acceleration values [in encoder units]
    int rampvelocity = 15000;
    int rampaccel = 16000;

    //Create PIDF Values
    int P = 1;
    int I = 0;
    int D = 0;
    int F = 0;

    //Inverse our sensors since they are out of phase by default
    steer.setSensorPhase(turningEncoderReversed);
    steer.setInverted(turningMotorInverse);
    //Configure motionmagic PID options for making our swerve turning clean
    steer.selectProfileSlot(0, kPIDLoopIdx);
    steer.configNominalOutputForward(0, kTimeoutMs);
    steer.configNominalOutputReverse(0, kTimeoutMs);
    steer.configPeakOutputForward(1, kTimeoutMs);
    steer.configPeakOutputReverse(-1, kTimeoutMs);
    steer.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);
    steer.configMotionCruiseVelocity(rampvelocity, kTimeoutMs);
    steer.configMotionAcceleration(rampaccel, kTimeoutMs);
    //steer.configFeedbackNotContinuous(false, kTimeoutMs);
    //steer.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    //Configure PIDF values with the integrated PID Controller
    steer.config_kF(kPIDLoopIdx, F, kTimeoutMs);
    steer.config_kP(kPIDLoopIdx, P, kTimeoutMs);
    steer.config_kI(kPIDLoopIdx, I, kTimeoutMs);
    steer.config_kD(kPIDLoopIdx, D, kTimeoutMs);

    //seed the encoder to use absolute positioning in relative mode
    initQuadrature();
  }

  private void setupDriveEncoder() {
    //clear any unwanted garbage off at startup
    drive.restoreFactoryDefaults();

    //convert position encoder value to change from rotations to metres
    driveEncoder.setPositionConversionFactor( ((Math.PI * ModuleConstants.wheelDiameterMetres) / (ModuleConstants.driveEncoderCPR * 6.67) )) ;

    //convert velocity from RPM to metres per second
    driveEncoder.setVelocityConversionFactor(ModuleConstants.driveVelocityConversion);

    //Configure PID values in the drive encoder
    // PID coefficients
    double kP = .2; 
    double kI = 0;
    double kD = 0; 
    double kIz = 0; 
    double kFF = 0.000015; 
    double kMaxOutput = 1; 
    double kMinOutput = -1;

    // set PID coefficients
    drivePID.setP(kP);
    drivePID.setI(kI);
    drivePID.setD(kD);
    drivePID.setIZone(kIz);
    drivePID.setFF(kFF);
    drivePID.setOutputRange(kMinOutput, kMaxOutput);    
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  private double unitsToDegrees(double units) {
    return units / 4096 * 360;
  }

  private double getAngle() {
    double angle = unitsToDegrees(steerEncoder.getPulseWidthPosition() & 0xFFF);
    System.out.println(name + " " + angle);
    return angle;
  }
  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {


    double velocity = state.speedMetersPerSecond * Constants.DriveConstants.speedLimit;

    //set the steer motor to the calculated degree using the PID Algorithm from CTRE
    steer.set(ControlMode.MotionMagic, state.angle.getDegrees() / 360 * 4096);

    //push data to the dashboard about the modules state
    SmartDashboard.putNumber(name + "Goal Angle", state.angle.getDegrees());
    SmartDashboard.putNumber(name + "Current Angle", getAngle());

    SmartDashboard.putNumber(name + " Goal Veloicty", state.speedMetersPerSecond);
    SmartDashboard.putNumber(name + " Current Veloicty", driveEncoder.getVelocity());

    //set the drive motor velocity using REV closed loop stuff
    drive.set(velocity);
  }

  /**
   * Zeros all the SwerveModule encoders.
   */
  public void resetEncoders() {
    //reset the distance driven by the drive motor to zero
    driveEncoder.setPosition(0);
  }
  
  public void initQuadrature() {
		/* get the absolute pulse width position */
		int pulseWidth = steerEncoder.getPulseWidthPosition();

		/**
		 * Mask out the bottom 12 bits to normalize to [0,4095],
		 * or in other words, to stay within [0,360) degrees 
		 */
    pulseWidth = pulseWidth & 0xFFF;
    
		/* Update Quadrature position */
		steerEncoder.setQuadraturePosition(pulseWidth, kTimeoutMs);
  }

  /**
   * Get the distance driven
   */
  public double getDistance() {
    return driveEncoder.getPosition();
  }
  /**
   * Stop both motors from spinning, sets their speeds to zero.
   */
  public void stop() {
    steer.set(ControlMode.PercentOutput, 0);
    drive.set(0);
  }

  /**
   * Reset the value of the steering encoder.
   */
  public void reset() {
    steer.setSelectedSensorPosition(0);
  }
}
