// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.armCommand;
import frc.robot.commands.intakeCommand;
import frc.robot.commands.shootCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final shooterSubsystem shooterSubsystem = new shooterSubsystem();
  private final intakeSubsystem intakeSubsystem = new intakeSubsystem(
    ScoringConstants.kDirectionalIntakeMotorPort, 
    ScoringConstants.kIntakeMotorPort,
    false, 
    false);
  private final armSubsystem armSubsystem = new armSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_scorerController= new CommandXboxController(OIConstants.kScorerControllerPort);
  private final Joystick m_joysticks = new Joystick(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * public RobotContainer() {
   * // Configure the trigger bindings
   * configureBindings();
   * }
   * 
   * /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -m_driverController.getLeftY(),//getRawAxis(OIConstants.kDriverXAxis),
        () -> -m_driverController.getLeftX(),//getRawAxis(OIConstants.kDriverYAxis), // The swerve drive auto github had x and y switched.
        () -> m_driverController.getRightX(),//(OIConstants.kDriverRotAxis),
        () -> !m_joysticks.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    configureBindings();

  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));
    Trigger aButton = m_scorerController.a();
    Trigger ybutton = m_scorerController.y();
    Trigger xbutton = m_scorerController.x();
    Trigger bbutton = m_scorerController.b();
      final shootCommand shoot = new shootCommand(shooterSubsystem);
      aButton.whileTrue(shoot);

      final intakeCommand intake = new intakeCommand(intakeSubsystem);
      ybutton.whileTrue(intake);

      final armCommand armUp = new armCommand(armSubsystem, ScoringConstants.armSpeed);
      xbutton.whileTrue(armUp);

      final armCommand armDown = new armCommand(armSubsystem, -ScoringConstants.armSpeed);
      bbutton.whileTrue(armDown);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
    //public Command getAutonomousCommand(int choice) {u
      /* 
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond
      , AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);
    
    edu.wpi.first.math.trajectory.Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0))
    ,
    List.of(
      new Translation2d(1,0),
      new Translation2d(1,-1)),
      new Pose2d(2,-1,Rotation2d.fromDegrees(180)),
    trajectoryConfig);
    
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    PIDController xController = new PIDController(AutoConstants.kPXController,0,0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController,yController,thetaController,swerveSubsystem::setModuleStates, swerveSubsystem);
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules())
    );
    */
    //}
   
}
