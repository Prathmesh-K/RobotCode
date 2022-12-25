// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. 
 */
public final class Constants {
  public static final class DriveConstants {

    public static final int kFrontLeft = 9;
    public static final int kRearLeft = 3;
    public static final int kFrontRight = 5;
    public static final int kRearRight = 7;

    //For Bolt Testing 
    /*
     public static final int kFrontLeft = 1;
     public static final int kRearLeft = 3;
     public static final int kFrontRight = 2;
     public static final int kRearRight = 4;*/

    public static final double kTrackwidthMeters = 0.8509;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    //Ramsete Controller values obtained from robot characterization tool. (REQUIRES MORE TESTING)
    public static final double ksVolts = 0.305;
    public static final double kvVoltSecondsPerMeter = 2.29;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0131;
    public static final double kPDriveVel = 8.5;


    //Gyro Command Constants
    public static final double kTurnP = 8.5;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double kTurnToleranceDeg = 4;
    public static final double kTurnRateToleranceDegPerS = 1;




  }

  public static final class ArmConstants {

    //CAN ID
    public static final int kArm = 17;

    //PID Controller Constants for Arm
    public static final double kP = .6;
    public static final double kI = 0;
    public static final double kD = .1;
    public static final double kIz = 0;
    public static final double kFF = 0.000156;

    //Trapezoidal Motion Profile Constants
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double kMaxVdown = 2500;
    public static final double kMaxVup = 4500;
    public static final double kMinV = 0;
    public static final double kMaxAup = 4000;
    public static final double kMaxAdown = 1000;
    public static final double kAllE = 0;

    //Encoder Constants Accounted for Gear Ratio
    public static final double kRotationsUp = 0.5;
    public static final double kRotationsDown = 27.5;

  }

  public static final class ClimberConstants {

    public static final int kClimb = 15;
    public static final int kClimb2 = 16;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0.000156;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double kMaxV = 3000;
    public static final double kMinV = 0;
    public static final double kMaxA = 2500;
    public static final double kAllE = 0;

    //On the Right Side increasing is down
    //On the Right Side decreasing is up
    public static final double kRightRotationsUp = -90.0;
    public static final double kRightRotationsDown = 0.0;
    //On the Left Side decreasing is down
    //On the Left Side increasing is up
    public static final double kLeftRotationsUp = 90.0;
    public static final double kLeftRotationsDown = 0.0;
    public static final double kResetRightSide = 85.0;
    public static final double kResetLeftSide = -90;
    public static final double kRotationsIdle = 0.0;

    

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final static class Conversions {

    public static double metersToFeet(double meters) {
      return meters * 3.28084;
    }

    public static double feetToMeters(double feet) {
      return feet * 0.3048;
    }

    public static double ticksToFeet(double ticks) {
      return (ticks / 4096) * (1 / 7.3529411) * 2 * Math.PI * (3 / 12);
    }

    public static double ticksToFeetPerSecond(double ticksPer100ms) {
      return (ticksPer100ms / 4096) * (1 / 7.3529411) * 2 * Math.PI * (3 / 12) * 10;
    }

    public static double ticksToMetersWheel(double ticks) {
      return (ticks / 4096.00) * (1.00 / 7.3529411) * 2 * Math.PI * (0.0762);
    }

    public static double ticksToMetersPerSecondWheel(double ticksPer100ms) {
      return (ticksPer100ms / 4096.00) * (1.00 / 7.3529411) * 2 * Math.PI * (0.0762) * 10;
    }

    public static double degreesToTicks(double angle) {
      return 4096.00 * (angle / 360.00);
    }

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = .2;
    public static final double kMaxAccelerationMetersPerSecondSquared =.1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static class kGains {

    public static final double kP = 0.000102;
    public static final double kI = 0.0;
    public static final double kD = 0.000438;
    public static final double kF = 0.0;
  }

  public static class GyroPID {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  public static final int kTimeoutMs = 10;
  public static final int kPIDLoopIdx = 0;
  public static final int kSlotIdx = 0;
  public static final double targetMeters = 2 * (6 * 2048 * 0.4787787204060999);
  //gear ratio is 6:1 
  public static final int smoothing = 4;
  //Smoothing is from 0 to 8

  public static class IntakeConstants {

    public static final int kIntake = 20;

  }
}
