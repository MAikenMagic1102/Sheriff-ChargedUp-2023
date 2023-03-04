// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.TalonFX;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX lowerArm = new TalonFX(13, "rio");
  private final TalonFX upperArm = new TalonFX(12, "rio");
  private DutyCycleOut lowerOutput = new DutyCycleOut(0);
  private DutyCycleOut upperOutput = new DutyCycleOut(0);

  TalonFXConfiguration lowerConfig = new TalonFXConfiguration();
  TalonFXConfiguration upperConfig = new TalonFXConfiguration();

  private boolean FOCenabled = true;
  private boolean OverrideBrakeNeutral = true;

  public Arm() {
    lowerConfig.Feedback.SensorToMechanismRatio = 36.0 / 1;
    upperConfig.Feedback.SensorToMechanismRatio = 9.0 / 1;

    var defaultConfig = new TalonFXConfiguration();
    TalonFXConfigurator lowerCfg = lowerArm.getConfigurator();
    TalonFXConfigurator upperCfg = upperArm.getConfigurator();

    lowerCfg.apply(defaultConfig);
    upperCfg.apply(defaultConfig);
    Timer.delay(0.5);
    
    upperCfg.apply(lowerConfig);
    lowerCfg.apply(upperConfig);
  }

  public void lowerArmOpenLoop(double demand){
    lowerOutput = new DutyCycleOut(demand * 0.4, FOCenabled, OverrideBrakeNeutral);
    lowerArm.setControl(lowerOutput);
  }

  public void upperArmOpenLoop(double demand){
    upperOutput = new DutyCycleOut(demand * 0.2, FOCenabled, OverrideBrakeNeutral);
    upperArm.setControl(upperOutput);
  }

  public void armsOpenLoop(double upperDemand, double lowerDemand){
    this.lowerArmOpenLoop(lowerDemand);
    this.upperArmOpenLoop(upperDemand);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Lower Arm Sensor POS", lowerArm.getPosition().getValue());
    SmartDashboard.putNumber("Upper Arm Sensor POS", upperArm.getPosition().getValue());

    SmartDashboard.putNumber("Lower Arm Current Draw", lowerArm.getTorqueCurrent().getValue());
    SmartDashboard.putNumber("Upper Arm Current Draw", upperArm.getTorqueCurrent().getValue());
  }
}
