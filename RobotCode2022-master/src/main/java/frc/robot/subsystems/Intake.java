//Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase { 
  WPI_TalonFX m_Intake = new WPI_TalonFX(Constants.IntakeConstants.kIntake);


  /** Creates a new Intake. */
  public Intake() {
    
  m_Intake.configFactoryDefault(Constants.kTimeoutMs);
  m_Intake.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void intakeIn() {
    m_Intake.set(0.5);
  }
  public void intakeOff() {
    m_Intake.set(0.0);
  }
  public void intakeOut() {
    m_Intake.set(-1.0);
  }


}
