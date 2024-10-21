// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.46;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 3.0; // percent per second (1 = 100%)
// 1.2 //  .8
//1.8 // 1.8
//2.0 // 2.5
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);
    //br 16, 5.5
    //bl 5.5, 3in
    //fl 3, 15
    //fr 15, 16
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      // weird
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), //fr? 15, 16
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //fl 15, 3
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //br 5.5 16
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //bl? 5.5 3

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 13; //FLD
    public static final int kBackLeftDrivingCanId = 3; //BLD
    public static final int kFrontRightDrivingCanId = 16; //FRD
    public static final int kBackRightDrivingCanId = 6; // BRD

    public static final int kFrontLeftTurningCanId = 12; // FLT
    public static final int kBackLeftTurningCanId = 2; //BLT
    public static final int kFrontRightTurningCanId = 15; // FRT
    public static final int kBackRightTurningCanId = 5; // BRT

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;//4.714285714285714

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps?
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
        PowerDistribution examplePD = new PowerDistribution(1, ModuleType.kRev);
        public static final int kDriverControllerPort = 0;
        public static final int kScorerControllerPort = 1;
        
        public static final boolean fieldRelative = true;

        // pretty sure this is better dependant on the controller
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  public static final class ScoringConstants{ 
    // Ethan's list of ports says that there is no directional intake, only left or right
    public static final int kDirectionalIntakeMotorPort = 20;
    public static final int kIntakeMotorPort = 4;
    public static final boolean directionalIntakeMotorReversed = false;
    public static final boolean intakeMotorReversed = false;

    public static final double intakeSpeed = 0.75;
    public static final double indexerSpeed = 0.60;

    public static final double triggerDeadBand = 0.3;
    
    public static final double speakerAngle = 0.15;
    public static final double stowAngle = 0.0;
    public static final double ampAngle = 0.340;
    public static final double intakeAngle = 0.942;

    public static final int kShootSetMotor = 19;//Indexer?
    public static final int kBackShootGoalMotor1 = 17;
    public static final int kFrontShootGoalMotor = 20;
    

    public static final int kleftArmMotor = 14;//Pivot?
    public static final int kRightArmMotor = 11;//Pivot?

    public static final double armMinSpeed = 0.01;
    public static final double armMaxSpeed = 0.15;

    public static final double speakerSpeed = 1;
    public static final double ampShotSpeedsBack = 0.330; //0.335;//325 //0.235//.3
    public static final double ampShotSpeedsFront = 0.28;//0.268;//265 //0.16;//233
    public static final double indexSpeed = 0.5;
    
    public static final int kAmpArmMotor = 18;
}
}