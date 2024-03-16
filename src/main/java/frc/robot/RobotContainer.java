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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.commands.ampShotCommand;
import frc.robot.commands.armCommand;
import frc.robot.commands.autoDriveCommand;
import frc.robot.commands.intakeCommand;
import frc.robot.commands.intakeDirectionCommand;
import frc.robot.commands.inverseIndex;
import frc.robot.commands.pivotArmSpecfic;
import frc.robot.commands.resetGyroCommand;
import frc.robot.commands.setArmSetPointCommand;
import frc.robot.commands.shootCommand;
import frc.robot.commands.stopShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
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

   private final SendableChooser<Command> autoChooser;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_scorerController = new CommandXboxController(OIConstants.kScorerControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
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
                OIConstants.fieldRelative, true), // be careful when false
            m_robotDrive));
    /*
 armSubsystem.setDefaultCommand(
        new RunCommand(
          () -> armSubsystem.moveArm( 
            -MathUtil.applyDeadband(m_scorerController.getLeftTriggerAxis(), OIConstants.kDeadband)), 
          armSubsystem));
          */

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
    //yDriverbutton.onTrue(resetGyro);
    //intake/indexers
    final inverseIndex reverse = new inverseIndex(intakeSubsystem);
    xScorerbutton.whileTrue(reverse);
    //new RunCommand(()->intakeSubsystem.rightStickIntake(m_scorerController.getRightY()), intakeSubsystem);

    final intakeCommand intake = new intakeCommand(intakeSubsystem);
    aScorerButton.whileTrue(intake);

    final intakeDirectionCommand changeDirection = new intakeDirectionCommand(intakeSubsystem);
    //scorerRightStick.onTrue(changeDirection);


    //shoot
    final shootCommand shoot = new shootCommand(shooterSubsystem);
    rightTrigger.whileTrue(shoot);
    
    final ampShotCommand ampShot = new ampShotCommand(shooterSubsystem);
    leftTrigger.whileTrue(ampShot);


    //arm pos- manual
    final armCommand armUp = new armCommand(armSubsystem, ScoringConstants.armMaxSpeed);
    dScorerUP.whileTrue(armUp);

    final armCommand armDown = new armCommand(armSubsystem, -ScoringConstants.armMaxSpeed);
    dScorerDOWN.whileTrue(armDown);

    final pivotArmSpecfic testArmSpecfic = new pivotArmSpecfic(armSubsystem, ScoringConstants.ampAngle);

    //arm pos- preset
    final pivotArmSpecfic speakerAngle = new pivotArmSpecfic(armSubsystem,ScoringConstants.speakerAngle);
    
    final pivotArmSpecfic stowArm = new pivotArmSpecfic(armSubsystem, ScoringConstants.stowAngle);

    final setArmSetPointCommand ampArm = new setArmSetPointCommand(armSubsystem, 0.0);
    aScorerButton.onTrue(ampArm);

    final setArmSetPointCommand intakeArm = new setArmSetPointCommand(armSubsystem, 0.13); //Placeholder! TODO: Change the angle.
    dScorerRIGHT.onTrue(intakeArm);

    final setArmSetPointCommand speakerArm = new setArmSetPointCommand(armSubsystem, 0.14); //Placeholder! TODO: Change the angle
    bScorerbutton.onTrue(speakerArm);//There might be a better button for this, IDK.
    //final pivotArmSpecfic intakeArm = new pivotArmSpecfic(armSubsystem, ScoringConstants.intakeAngle);
      
  }
//public Command getTestAutoCommand(){ 
  
   /*  
    return new SequentialCommandGroup(
      new InstantCommand(() -> shooterSubsystem.shoot()
    ));
    break;
    */
    //default:
    //return new SequentialCommandGroup( 
    //  new InstantCommand(() -> m_robotDrive.drive(0.5,0.0,0.0, false, true))
    //);

    // case(0):
    // return new SequentialCommandGroup(
    //   new InstantCommand(() -> DriveSubsystem.drive())
    // );
    
  
//}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

    //Load an autobuilder from autobuilder (autobuilder is in DriveSubsystem)
    public Command getAutonomousCommand1() 
    {
      return new PathPlannerAuto("Example Auto");
    }

  public Command getAutonomousCommand() 
  {
    return autoChooser.getSelected();
  }
  
    
  //public Command getAutonomousCommand() {
    /*
    // Create config for trajectory

      // AutoBuilder PathAuto = new AutoBuilder(
      // AutoConstants.kMaxSpeedMetersPerSecond,
      // AutoConstants.kMaxAccelerationMetersPerSecondSquared);

   
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
     */
    // Run path following command, then stop at the end.
    //---return Commands.swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)); //why swervecontroller command AND THEN
    //sequencev -//repearting sequence
    //parallel
    //race-

    //shoot has a premade stop command, dont know if that acounts fo rit
    //return Commands.run(shoot, shooterSubsystem);
    //return Commands.runOnce(()-> shoot, shooterSubsystem);
    //return Commands.sequence(shoot).andThen(intake).andThen(()-> m_robotDrive.drive(0.2,0.0,0.0,OIConstants.fieldRelative,true)).until(()->new WaitCommand(3).isFinished()).andThen(()-> m_robotDrive.drive(0,0,0,OIConstants.fieldRelative,true)).andThen(stopShoot); //problem, drive doesnt stop
    //autoDriveCommand backwards = new autoDriveCommand(m_robotDrive);

    /* return Commands.runOnce(
      () -> shooterSubsystem.shoot(),
      shooterSubsystem).andThen(new WaitCommand(2))
      .andThen(()->intakeSubsystem.enableIntake(0.5),
      intakeSubsystem).andThen(new WaitCommand(5))
      .andThen(new WaitCommand(1)).andThen(()-> shooterSubsystem.endShoot(),shooterSubsystem)
      .andThen(()-> m_robotDrive.drive(0.0,0.5,0.0,OIConstants.fieldRelative,true),m_robotDrive);
      //.andThen(backwards);
 */
    
  }