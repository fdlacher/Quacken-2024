package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.ModuleConstants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final PIDController turningPIDController;

    //private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed,
            boolean turnMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset,
            boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        //absoluteEncoder = new AnalogInput();

        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();
        
        //converstion factor
        driveEncoder.setPositionConversionFactor(ModuleConstants.KDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MetersPerSec);
        turnEncoder.setPositionConversionFactor(ModuleConstants.KDriveEncoderRot2Meter);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MetersPerSec);

        turningPIDController = new PIDController(ModuleConstants.KPTurning, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI); // informs that its a circle

        resetEncoders();
    }

    public double getDrivePostion() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }
    
    /*
    public double getAbsoluteEncoderRad (){
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle *= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }
    */
    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(0);
    }
   
    
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePostion(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.0001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);//get state from state and assigns it to motor
        turnMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
