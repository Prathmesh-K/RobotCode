// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.camera;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.BasicAuto;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.commands.GyroTurn;
import frc.robot.commands.MotionMagicDrive;
import frc.robot.commands.toggleFastMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.swing.SwingConstants;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
   private final DriveSubsystem m_robotDrive = new DriveSubsystem();
   private final climber m_Climber = new climber();
   private final Intake m_Intake = new Intake(); 
   private final Arm m_Arm = new Arm();
   private final Pneumatics m_Pneumatics = new Pneumatics();
   private final SlewRateLimiter Left = new SlewRateLimiter(3);
    private final SlewRateLimiter Right = new SlewRateLimiter(3);
    private final camera m_camera = new camera();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverController2 = new XboxController(OIConstants.kDriverControllerPort);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
  

    configureButtonBindings(); { 
        new JoystickButton(m_driverController, Button.kLeftBumper.value).toggleWhenPressed
        (new StartEndCommand (m_Intake::intakeIn,m_Intake::intakeOff,m_Intake));
      }

    configureButtonBindings(); { 
        new JoystickButton(m_driverController, Button.kRightBumper.value).toggleWhenPressed
        (new StartEndCommand (m_Intake::intakeOut,m_Intake::intakeOff,m_Intake));
      }
      configureButtonBindings(); { 
        new JoystickButton(m_driverController, Button.kB.value).whenPressed
        (new StartEndCommand (m_Arm::ArmDown,m_Arm::ArmUp,m_Arm));
      }
      configureButtonBindings(); { 
         new JoystickButton(m_driverController, Button.kA.value).whenPressed
         (new StartEndCommand (m_Arm::ArmUp,m_Arm::ArmDown,m_Arm));
      }
      configureButtonBindings(); { 
        new JoystickButton(m_driverController, Button.kStart.value).whenPressed
        (new toggleFastMode( m_robotDrive));
     }

      //  configureButtonBindings(); { 
      //   new JoystickButton(m_driverController, Button.kX.value).whenPressed
      //   (new StartEndCommand (m_Pneumatics::ClimberAngled,m_Pneumatics::ClimberStraight,m_Pneumatics));
      // }
      configureButtonBindings(); { 
        new JoystickButton(m_driverController, Button.kY.value).whenPressed
        (new StartEndCommand (m_Climber::climberUp,m_Climber::climberDown,m_Climber));
      }
      configureButtonBindings(); { 
        new JoystickButton(m_driverController, Button.kX.value).whenPressed
        (new StartEndCommand (m_Climber::climberDown,m_Climber::climberUp,m_Climber));
      }
     
    
      // m_Climber.setDefaultCommand(
      //   new RunCommand(
      //       () -> {
      //         int ClimberCount = 0;
      //         double kAngle = m_driverController.getPOV();
      //         if (kAngle == 180 && ClimberCount <= Constants.ClimberConstants.kRotationsUp) {
      //           m_Climber.moveclimber(ClimberCount);
      //           ClimberCount++;
                
      //         }
      //         if (kAngle == 0 && ClimberCount >= Constants.ClimberConstants.kRotationsDown) {
      //           m_Climber.moveclimber(ClimberCount);
      //           ClimberCount--;
               
      //         }
      //       },m_Climber));
      
   
      
   
      
    
  
    // Configure default commands
    // Set the default drive command to split-stick arcade drive

        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        // new RunCommand( ()->


        // );
        m_robotDrive.setDefaultCommand(
          // new RunCommand(() ->
          //     m_robotDrive.tankDrive(
          //         -Left.calculate(m_driverController.getLeftY()),
          //         -Right.calculate(m_driverController.getRightY())),
          //         m_robotDrive));
          new ArcadeDriveCommand(
                          m_robotDrive,
                          () -> m_driverController.getLeftY(),
                          () -> m_driverController.getRightX())
          );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
     

      // Set the default drive command to split-stick arcade drive
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.
     
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  

  
    
   //Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = 
            new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            4);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond, 
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

     String RedBallDriveOut = "paths/Red Ball Drive Out/1.wpilib.json";
     Trajectory RedBallDriveOutTrajectory = new Trajectory();

     try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(RedBallDriveOut);
      RedBallDriveOutTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + RedBallDriveOut, ex.getStackTrace());
   }

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 0), new Translation2d(1.5, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand exampleramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts, //static friction gain
                DriveConstants.kvVoltSecondsPerMeter, //velocity friction gain
                DriveConstants.kaVoltSecondsSquaredPerMeter), //acceleration friction gain
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0), //PID
            new PIDController(DriveConstants.kPDriveVel, 0, 0), //PID
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);

            RamseteCommand RedBallDriveOutRamsete =
            new RamseteCommand(
                RedBallDriveOutTrajectory,
                m_robotDrive::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                m_robotDrive::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0), //p should be more than 0 
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts,
                m_robotDrive);

   // Reset odometry to the starting pose of the trajectory.
   m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

   //Run path following command, then stop at the end.
  // return exampleramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

  
   // new DriveForwardCommand(m_robotDrive, .05, .05).withTimeout(1),
    //new GyroTurn(90, m_robotDrive).withTimeout(7)

    
    return new SequentialCommandGroup(
      new RunCommand(m_Intake::intakeOut, m_Intake).withTimeout(3),
      new RunCommand(m_Intake::intakeOff, m_Intake).withTimeout(1),
      new DriveForwardCommand(m_robotDrive, .15, .15).withTimeout(3));

      //new MotionMagicDrive(m_robotDrive, 2.19)); 
      // new GyroTurn(90, m_robotDrive),
     // new MotionMagicDrive(m_robotDrive, 2))
    // .andThen());
                                
    
    
    // new WaitCommand(3),
    // new DriveForwardCommand(m_robotDrive).withTimeout(4)

    // new DriveStopCommand(driveSubsystem)



  //  return RedBallDriveOutRamsete.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

   


  //  return  new SequentialCommandGroup( new RunCommand(m_robotDrive::MagicTaxi));
     
    
    // Run path following command, then stop at the end.
 //   return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

   
    
  }
}