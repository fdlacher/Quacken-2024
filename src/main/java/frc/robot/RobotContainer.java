// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.commands.armCommand;
import frc.robot.commands.intakeCommand;
import frc.robot.commands.pivotArmSpecfic;
import frc.robot.commands.shootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final shooterSubsystem shooterSubsystem = new shooterSubsystem();

   private final armSubsystem armSubsystem = new armSubsystem();
   private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_scorerController = new CommandXboxController(OIConstants.kScorerControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDeadband),
                true, true),
            m_robotDrive));
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
    new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive);//unknown if this will register...

    Trigger aButton = m_scorerController.a();//Bumber to speaker
    Trigger ybutton = m_driverController.y();//Bumper to amp
    Trigger xbutton = m_driverController.x();//Intake
    Trigger bbutton = m_driverController.b();//Arm Down
    Trigger dUP = m_scorerController.povUp();
    Trigger dDOWN = m_scorerController.povDown();
    Trigger dRIGHT = m_scorerController.povRight();
    Trigger dLEFT = m_scorerController.povDown();

    final pivotArmSpecfic speakerAngle = new pivotArmSpecfic(
      armSubsystem, 
      ScoringConstants.speakerAngle
      );
    final pivotArmSpecfic stowArm = new pivotArmSpecfic(armSubsystem, ScoringConstants.stowAngle);
    bbutton.whileTrue(stowArm);
    final pivotArmSpecfic ampArm = new pivotArmSpecfic(armSubsystem, ScoringConstants.ampAngle);
    ybutton.whileTrue(ampArm);

    final shootCommand shoot = new shootCommand(shooterSubsystem);
      aButton.whileTrue(shoot);

      final intakeCommand intake = new intakeCommand(intakeSubsystem);
      xbutton.whileTrue(intake);

      final armCommand armUp = new armCommand(armSubsystem, ScoringConstants.armMaxSpeed);
      dUP.whileTrue(armUp);

      final armCommand armDown = new armCommand(armSubsystem, -ScoringConstants.armMaxSpeed);
      dDOWN.whileTrue(armDown);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
