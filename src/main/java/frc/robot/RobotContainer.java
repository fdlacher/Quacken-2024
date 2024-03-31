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
//import frc.robot.commands.intakeCommand;
import frc.robot.commands.shootCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.armSubsystem;
//import frc.robot.subsystems.intakeSubsystem;
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
  // private final intakeSubsystem intakeSubsystem = new intakeSubsystem(
  //   ScoringConstants.kDirectionalIntakeMotorPort, 
  //   ScoringConstants.kIntakeMotorPort,
  //   false, 
  //   false);
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
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    
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
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDeadband+.2),
                true, true), // be careful when false
            m_robotDrive));
            
    /*
 armSubsystem.setDefaultCommand(
        new RunCommand(
          () -> armSubsystem.moveArm( 
            -MathUtil.applyDeadband(m_scorerController.getLeftTriggerAxis(), OIConstants.kDeadband)), 
          armSubsystem));
          */
          
  //Is this redundant?
  shooterSubsystem.setDefaultCommand(
      new RunCommand(
      () -> shooterSubsystem.shoot(
        -MathUtil.applyDeadband(m_scorerController.getRightTriggerAxis(), OIConstants.kDeadband))
      ,shooterSubsystem));
      
      
    intakeSubsystem.setDefaultCommand(
      new RunCommand(()-> intakeSubsystem.rightStickIntake(-MathUtil.applyDeadband(m_scorerController.getRightY(), OIConstants.kDeadband)), intakeSubsystem)
    );
    // set point-constantly be running
    
    
    armSubsystem.setDefaultCommand(new RunCommand(()-> armSubsystem.moveArm(ScoringConstants.armMaxSpeed, 
                                  -MathUtil.applyDeadband(m_scorerController.getLeftY(), OIConstants.kDeadband)),
                                   armSubsystem));
                                   
                                   

    //armSubsystem.setDefaultCommand(new RunCommand(()-> armSubsystem.goToSetPoint(),armSubsystem));

    //Auto Commands
    //these are used for pathplanner and need the name that is used in pathPlanner.
  
    final speakerShotCommand shoot = new speakerShotCommand(shooterSubsystem);
    final ampShotCommand ampShoot = new ampShotCommand(shooterSubsystem);
    final intakeCommand intake = new intakeCommand(intakeSubsystem);
    final resetGyroCommand reset = new resetGyroCommand(m_robotDrive);
    NamedCommands.registerCommand("Shoot", shoot);
    NamedCommands.registerCommand("ampShoot", ampShoot);
    NamedCommands.registerCommand("intake", intake);
    NamedCommands.registerCommand("Reset", reset);


    
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

    //Trigger aDriverButton = m_scorerController.a();//Bumber to speaker
    Trigger yDriverbutton = m_driverController.y();//Reset gyro
    //Trigger xDriverbutton = m_driverController.x();//Intake
    //Trigger bDriverbutton = m_driverController.b();//Arm Down
    //Trigger dDriverUP = m_driverController.povUp(); //pivot arm
    //Trigger dDriverDriverDOWN = m_driverController.povDown(); //pivot arm
    //Trigger dDriverRIGHT = m_driverController.povRight();
    //Trigger dDriverLEFT = m_driverController.povDown();


    //Trigger scorerRightStick = m_scorerController.rightStick();//flip intake - press it

    Trigger aScorerButton = m_scorerController.a();
    Trigger yScorerbutton = m_scorerController.y();
    Trigger bScorerbutton = m_scorerController.b();
    Trigger xScorerbutton = m_scorerController.x();
    
    Trigger leftTrigger = m_scorerController.leftTrigger(ScoringConstants.triggerDeadBand); //amp shot
    Trigger rightTrigger = m_scorerController.rightTrigger(ScoringConstants.triggerDeadBand); //shoot

    Trigger dScorerUP = m_scorerController.povUp();  // intake angle -//temp arm up
    Trigger dScorerDOWN = m_scorerController.povDown(); //speaker angle -//temp arm down
    Trigger dScorerRIGHT = m_scorerController.povRight(); //stow arm
    Trigger dScorerLEFT = m_scorerController.povLeft(); // amp angle

    
    final resetGyroCommand resetGyro = new resetGyroCommand(m_robotDrive);
    yDriverbutton.onTrue(resetGyro);
    //intake/indexers
    final inverseIndex reverse = new inverseIndex(intakeSubsystem);
    xScorerbutton.whileTrue(reverse);
    //new RunCommand(()->intakeSubsystem.rightStickIntake(m_scorerController.getRightY()), intakeSubsystem);

      // final intakeCommand intake = new intakeCommand(intakeSubsystem);
      // ybutton.whileTrue(intake);

      final armCommand armUp = new armCommand(armSubsystem, ScoringConstants.armSpeed);
      dUP.whileTrue(armUp);

    final armCommand armDown = new armCommand(armSubsystem, -ScoringConstants.armMaxSpeed);
    dScorerDOWN.whileTrue(armDown);

    final pivotArmSpecfic testArmSpecfic = new pivotArmSpecfic(armSubsystem, ScoringConstants.ampAngle);

    //arm pos- preset
    final pivotArmSpecfic speakerAngle = new pivotArmSpecfic(armSubsystem,ScoringConstants.speakerAngle);
    
    final pivotArmSpecfic stowArm = new pivotArmSpecfic(armSubsystem, ScoringConstants.stowAngle);

    final setArmSetPointCommand ampArm = new setArmSetPointCommand(armSubsystem, 0.0);
    aScorerButton.onTrue(ampArm);

    final setArmSetPointCommand intakeArm = new setArmSetPointCommand(armSubsystem, 0.13); //Placeholder! TODO: Change the angle.
    //dScorerRIGHT.onTrue(intakeArm);

    final setArmSetPointCommand speakerArm = new setArmSetPointCommand(armSubsystem, 0.14); //Placeholder! TODO: Change the angle
    //bScorerbutton.onTrue(speakerArm);//There might be a better button for this, IDK.
    //final pivotArmSpecfic intakeArm = new pivotArmSpecfic(armSubsystem, ScoringConstants.intakeAngle);
    
    final ampArmPivotBasicCommand ampMoveArmUp = new ampArmPivotBasicCommand(ampArmSubsystem,0.05);
    bScorerbutton.onTrue(ampMoveArmUp);

    final ampArmPivotBasicCommand ampMoveArmDown = new ampArmPivotBasicCommand(ampArmSubsystem, -0.05);
    yScorerbutton.onTrue(ampMoveArmDown);
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
