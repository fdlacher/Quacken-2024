package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

//import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

//module should contain a drive motor, a direction motor, and an encoder
class SwerveModule{
    
    private CANSparkMax Drive_Motor;
    private CANSparkMax Direction_Motor;
    // private inset encoder here?

    // state VAraibles
    private SwerveModuleState currentState;
    private SwerveModuleState desiredState;

    //we need TWO PID controllers
    PIDController DrivePIDController;
    PIDController TurnPIDController; 

    public SwerveModule(
    int Drive_Motor_Port,
    int Direction_Motor_Port
    ){
        Drive_Motor = new CANSparkMax(Drive_Motor_Port, MotorType.kBrushless);
        Direction_Motor = new CANSparkMax(Direction_Motor_Port, MotorType.kBrushless);
        
        currentState = new SwerveModuleState();

        DrivePIDController = new PIDController(0.1,0, 0);
        TurnPIDController = new PIDController(0.1, 0, 0);
    }

    public SwerveModuleState getState(){
        return desiredState;
    } 

    public void setDesiredState(SwerveModuleState state){
        desiredState = state;
    
        Drive_Motor.setVoltage(1);
        Direction_Motor.setVoltage(2);
    }
}



public class SwerveSubsystem extends SubsystemBase{
    
    SwerveModule FrontLeftModule = new SwerveModule(RobotMap.driveConstants.K_Front_Left_Drive_Motor,RobotMap.driveConstants.K_Front_Left_Turn_Motor);
    SwerveModule FrontRightModule = new SwerveModule(RobotMap.driveConstants.K_Front_Right_Drive_Motor,RobotMap.driveConstants.K_Front_Right_Turn_Motor);
    SwerveModule BackLeftModule = new SwerveModule(RobotMap.driveConstants.K_Back_Left_Drive_Motor,RobotMap.driveConstants.K_Back_Left_Turn_Motor);
    SwerveModule BackRightModule = new SwerveModule(RobotMap.driveConstants.K_Back_Right_Drive_Motor,RobotMap.driveConstants.K_Back_Right_Turn_Motor);


    //define kinimaatiec object
    double chassisWidth = Units.inchesToMeters(24);//inches
    double chassisLength = Units.inchesToMeters(24); // inches

    // defined location of wheels relative to center
    Translation2d FrontLeftLocation = new Translation2d(chassisLength /2, chassisWidth /2);
    Translation2d FrontRightLocation = new Translation2d(chassisLength /2, -chassisWidth /2);
    Translation2d BackLeftLocation = new Translation2d(-chassisLength /2, chassisWidth /2);
    Translation2d BackRightLocation = new Translation2d(-chassisLength /2, -chassisWidth /2);

    SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(
        FrontLeftLocation,
        FrontRightLocation,
        BackLeftLocation,
        BackRightLocation
    );

    // controller refrence
    CommandXboxController controller;

    public SwerveSubsystem(){
        System.out.println("SwerveSubsystem created");
    }
        public SwerveSubsystem(CommandXboxController io){
        System.out.println("SwerveSubsystem created");
        controller = io;
    }

    public void setChassisSpeed(ChassisSpeeds desiredSpeed){
        //get desired state'
        SwerveModuleState[] newStates = Kinematics.toSwerveModuleStates(desiredSpeed);

        FrontLeftModule.setDesiredState(newStates[0]);
        FrontRightModule.setDesiredState(newStates[1]);
        BackLeftModule.setDesiredState(newStates[2]);
        BackRightModule.setDesiredState(newStates[3]);

    }

    @Override
    public void periodic() {
        //sends data to smart dashboard(frc driver station)

        //get the x and y values from left joystick
        ChassisSpeeds newDesiredSpeed = new ChassisSpeeds(
            -controller.getLeftY(),
            // pushing left, ask robot to move left
            -controller.getLeftX(),
            // pushing left, ask robot to move left
            controller.getRightX()
        );

        setChassisSpeed(newDesiredSpeed);

        // FL FR BL BR
        double lodgingState[] = {
            FrontLeftModule.getState().angle.getDegrees(),
            FrontLeftModule.getState().speedMetersPerSecond,

            FrontRightModule.getState().angle.getDegrees(),
            FrontRightModule.getState().speedMetersPerSecond,

            BackLeftModule.getState().angle.getDegrees(),
            BackLeftModule.getState().speedMetersPerSecond,

            BackRightModule.getState().angle.getDegrees(),
            BackRightModule.getState().speedMetersPerSecond
        };

        //send data to the smartBoard
        
        SmartDashboard.putNumberArray("SwerveModuleStates", lodgingState);
    
    }
}
