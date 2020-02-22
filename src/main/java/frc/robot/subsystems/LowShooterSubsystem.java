/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LowShooterSubsystem extends SubsystemBase {
  private static LowShooterSubsystem instance;
  private final TalonSRX shootMotor = new TalonSRX(Constants.shootMotorID);
  //private final TalonSRX shootMotorRight = new TalonSRX(Constants.shootLeftMotorID);
  private final DoubleSolenoid actuatorSolenoid = new DoubleSolenoid(Constants.actuatorModuleID, 
    Constants.actuatorPistonExtendID, Constants.actuatorPistonRetractID);
  //private Ultrasonic detectFuel = new Ultrasonic(Constants.detectFuelTriggerID, Constants.detectFuelEchoID);
  private DigitalInput detectFuel = new DigitalInput(Constants.digitalSensorIntakeID);
  private DigitalInput stopFuel = new DigitalInput(Constants.digitalSensorGateID);
  //private AnalogInput sensor = new AnalogInput (Constants.sensorID);
  // IR was to unstable, readings varied too much.
  private double speed = 0;
  private int fuelState = 0;
  // private int fuelCount = 0;
  private boolean isExtended;
  private boolean stopCollection = false;
  private boolean shootingMode = false;

  //private double speedRight = 0;
  /**
   * Creates a new ShooterSubsystem.
   */
  private LowShooterSubsystem() {
    shootMotor.set(ControlMode.PercentOutput, 0.5);

    extendPistonActuator();
    //for Ultrasonic
    //detectFuel.setAutomaticMode(true);

    //detectFuel.getRangeInches();
  }
  public static LowShooterSubsystem getInstance() {
    if(instance == null){
      instance = new LowShooterSubsystem();
    }
    return instance;
  }
  public void shoot(double newSpeed){
    speed = -newSpeed;
    shootingMode = true;
  }
  public void stop(){
    speed = 0;
    shootingMode = false;
  }
  public void extendPistonActuator() {
    isExtended = true;
    actuatorSolenoid.set(Value.kForward);
  }
  public void retractPistonActuator() {
    if (!RobotContainer.isEndgame()){
      isExtended = false;
      actuatorSolenoid.set(Value.kReverse);
    }
  }
  public void finishShooting(){
    shootMotor.set(ControlMode.PercentOutput, 0);
    speed = 0;
    //fuelCount = 0;
    fuelState = 0;
    shootingMode = false;
  }
  public void runMotor() {
    shootMotor.set(ControlMode.PercentOutput, 0.5);

  }
  //public void runMotors() {
    //shootMotor.set(ControlMode.PercentOutput, speed);
    //shootMotorRight.set(ControlMode.PercentOutput, speedRight);
  //}
  @Override
  public void periodic() {
    // shootMotor.set(ControlMode.PercentOutput, 1.0);
    if (RobotContainer.isEndgame()){
      //shootMotor.set(ControlMode.PercentOutput,0);
      if(!isExtended){
        extendPistonActuator();
      }
    } else {
      //shootMotor.set(ControlMode.PercentOutput, speed);
    }
    if (shootingMode){
      shootMotor.set(ControlMode.PercentOutput, speed);
      return;
    }
    //SmartDashboard.putNumber("Ultrasonic Sensor", detectFuel.getRangeInches());
    SmartDashboard.putBoolean("Intake Digital Sensor", detectFuel.get());
    SmartDashboard.putBoolean("Gate Digital Sensor", stopFuel.get());
    //SmartDashboard.putNumber("IR Sensor", sensor.getAverageVoltage());
    //double distance = sensor.getAverageVoltage();

    //if Ultrasonic change to double, if digital change to boolean
    boolean fuelIntake = !detectFuel.get(); //Sensor returns false if ball detected
    boolean stopIntake = !stopFuel.get();
    //if ultrasonic change to distance < 8.0 and distance > 8.5, if digital change to ==false and ==true
    //sensor at front of low shooter
    switch (fuelState){
      case 0: 
        if (fuelIntake && !stopIntake /*&& fuelCount < 4*/) {
          fuelState++;
        } else {
          shootMotor.set(ControlMode.PercentOutput, 0);
        }
        break;
      case 1:
        if (!fuelIntake || stopIntake) {
          fuelState++;
        } else {
          shootMotor.set(ControlMode.PercentOutput, -0.5);
        }
        break;
      case 2:
        //fuelCount++;
        shootMotor.set(ControlMode.PercentOutput, 0);
        fuelState = 0;
        break;
    }
    //shootMotorRight.set(ControlMode.PercentOutput, speedRight);
    // This method will be called once per scheduler run
  }
  
}
