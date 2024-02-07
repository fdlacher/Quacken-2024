package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import com.kauailabs.navx.frc.AHRS; get gyro
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.K_Front_Left_Drive_Motor_Port,
        DriveConstants.K_Front_Left_Turn_Motor_Port,
        DriveConstants.K_Front_Left_Drive_Encoder_Reversed,
        DriveConstants.K_Front_Left_Turn_Encoder_Reversed,
        DriveConstants.K_Front_Left_Drive_Absolute_Encoder_Port,
        DriveConstants.K_Front_Left_Drive_Absolute_Encoder_Offset_Rad,
        DriveConstants.K_Front_Left_Drive_Absolute_Encoder_Reversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.K_Front_Right_Drive_Motor_Port,
        DriveConstants.K_Front_Right_Turn_Motor_Port,
        DriveConstants.K_Front_Right_Drive_Encoder_Reversed,
        DriveConstants.K_Front_Right_Turn_Encoder_Reversed,
        DriveConstants.K_Front_Right_Drive_Absolute_Encoder_Port,
        DriveConstants.K_Front_Right_Drive_Absolute_Encoder_Offset_Rad,
        DriveConstants.K_Front_Right_Drive_Absolute_Encoder_Reversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.K_Back_Left_Drive_Motor_Port,
        DriveConstants.K_Back_Left_Turn_Motor_Port,
        DriveConstants.K_Back_Left_Drive_Encoder_Reversed,
        DriveConstants.K_Back_Left_Turn_Encoder_Reversed,
        DriveConstants.K_Back_Left_Drive_Absolute_Encoder_Port,
        DriveConstants.K_Back_Left_Drive_Absolute_Encoder_Offset_Rad,
        DriveConstants.K_Back_Left_Drive_Absolute_Encoder_Reversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.K_Back_Right_Drive_Motor_Port,
        DriveConstants.K_Back_Right_Turn_Motor_Port,
        DriveConstants.K_Back_Right_Drive_Encoder_Reversed,
        DriveConstants.K_Back_Left_Turn_Encoder_Reversed,
        DriveConstants.K_Back_Right_Drive_Absolute_Encoder_Port,
        DriveConstants.K_Back_Right_Drive_Absolute_Encoder_Offset_Rad,
        DriveConstants.K_Back_Right_Drive_Absolute_Encoder_Reversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0));

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
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
