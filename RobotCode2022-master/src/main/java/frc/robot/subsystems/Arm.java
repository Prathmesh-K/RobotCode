// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  //Declaring the arm motor, PID controller, and encoder
  private CANSparkMax m_arm = new CANSparkMax(Constants.ArmConstants.kArm, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController = m_arm.getPIDController();
  private RelativeEncoder m_encoder = m_arm.getEncoder();

  public Arm() {
    // m_arm.restoreFactoryDefaults();

  }

  public void ArmDown() {
    //Setting the PID values used to position the arm at the down position and to hold it there through constant adjustments
    m_pidController.setP(Constants.ArmConstants.kP);
    m_pidController.setI(Constants.ArmConstants.kI);
    m_pidController.setD(Constants.ArmConstants.kD);

    //Setting the feed forward values which account for the weight of the arm and the friction of the arm and motor 
    m_pidController.setIZone(Constants.ArmConstants.kIz);
    m_pidController.setFF(Constants.ArmConstants.kFF);

    //Setting the values for the trapazoidal motion profile which smooths out the motion of the arm through the use of a trapezoid shaped velocity curve.
    m_pidController.setOutputRange(Constants.ArmConstants.kMinOutput, Constants.ArmConstants.kMaxOutput);
    m_pidController.setSmartMotionMaxVelocity(Constants.ArmConstants.kMaxVdown, 0);
    m_pidController.setSmartMotionMinOutputVelocity(Constants.ArmConstants.kMinV, 0);
    m_pidController.setSmartMotionMaxAccel(Constants.ArmConstants.kMaxAdown, 0);
    m_pidController.setSmartMotionAllowedClosedLoopError(Constants.ArmConstants.kAllE, 0);
    m_pidController.setReference(Constants.ArmConstants.kRotationsDown, CANSparkMax.ControlType.kSmartMotion);
  }

  public void ArmUp() {
    //Setting the PID values used to position the arm at the up position and to hold it there through constant adjustments
    m_pidController.setP(Constants.ArmConstants.kP);
    m_pidController.setI(Constants.ArmConstants.kI);
    m_pidController.setD(Constants.ArmConstants.kD);

    //Setting the feed forward values which account for the weight of the arm and the friction of the arm and motor
    m_pidController.setIZone(Constants.ArmConstants.kIz);
    m_pidController.setFF(Constants.ArmConstants.kFF);

    //Setting the values for the trapazoidal motion profile which smooths out the motion of the arm through the use of a trapezoid shaped velocity curve.
    m_pidController.setOutputRange(Constants.ArmConstants.kMinOutput, Constants.ArmConstants.kMaxOutput);
    m_pidController.setSmartMotionMaxVelocity(Constants.ArmConstants.kMaxVup, 0);
    m_pidController.setSmartMotionMinOutputVelocity(Constants.ArmConstants.kMinV, 0);
    m_pidController.setSmartMotionMaxAccel(Constants.ArmConstants.kMaxAup, 0);
    m_pidController.setSmartMotionAllowedClosedLoopError(Constants.ArmConstants.kAllE, 0);
    m_pidController.setReference(Constants.ArmConstants.kRotationsUp, CANSparkMax.ControlType.kSmartMotion);
  }

  @Override
  public void periodic() {

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("ArmEncoderPosition", m_encoder.getPosition());
    SmartDashboard.putData(this);
  }
}
