package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
//import com.kauailabs.navx.frc.AHRS; get gyro
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.util.sendable.Sendable;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
//import java.lang.AutoCloseable;

import java.lang.System.Logger;

import com.kauailabs.navx.frc.AHRS; // dnag it, found it the same time you did

//import edu.wpi.first.wpilibj.SPI.Port;

public class SwerveSubsystem extends SubsystemBase {
    public double[] backHand = {0,0,0,0,0,0,0,0};

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort, // abso enc port
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed); // abs enc rev

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    /*
     * private final SwerveDriveOdometry odometer = new
     * SwerveDriveOdometry(DriveConstants.kDriveKinematics,
     * new Rotation2d(0), new SwerveModule[] {
     * frontLeft.getDrivePostion(),
     * frontRight,
     * backLeft,
     * backRight}); // place swerve module positions
     */

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            gyro.getRotation2d(),
            getSwerveModulePosition(),
            new Pose2d()
    // getPose()// Might not be the right method, not sure. doesnt work because
    // odometer hasnt been made yet
    );

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();// this might work
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);// This might work
    }

    public double testGyro() {
        return gyro.getAngle();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getSwerveModulePosition(), pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getSwerveModulePosition());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putNumberArray("SwerveModule", backHand);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public SwerveModulePosition[] getSwerveModulePosition() {

        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
        backHand[0] = desiredStates[0].speedMetersPerSecond;
        backHand[1] = desiredStates[0].angle.getRadians();

        backHand[2] = desiredStates[1].speedMetersPerSecond;
        backHand[3] = desiredStates[1].angle.getRadians();

        backHand[4] = desiredStates[2].speedMetersPerSecond;
        backHand[5] = desiredStates[2].angle.getRadians();

        backHand[6] = desiredStates[3].speedMetersPerSecond;
        backHand[7] = desiredStates[3].angle.getRadians();

    }

    public SwerveModuleState[] getSwerveModuleStateArray() {
        SwerveModuleState[] swerveModuleStates = {
            frontLeft.getState(), 
            frontRight.getState(), 
            backLeft.getState(), 
            backRight.getState()}; 
        return swerveModuleStates;
    }
}
