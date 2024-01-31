package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.driveConstants;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.lang.AutoCloseable;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Drive extends SubsystemBase {
      private static CANSparkMax FRONTLEFT_DRIVE_SPEED_MOTOR;
      private static CANSparkMax FRONTRIGHT_DRIVE_SPEED_MOTOR;
      private static CANSparkMax BACKLEFT_DRIVE_SPEED_MOTOR;
      private static CANSparkMax BACKRIGHT_DRIVE_SPEED_MOTOR;

      private static CANSparkMax FRONTLEFT_DRIVE_DIRECTION_MOTOR;
      private static CANSparkMax FRONTRIGHT_DRIVE_DIRECTION_MOTOR;
      private static CANSparkMax BACKLEFT_DRIVE_DIRECTION_MOTOR;
      private static CANSparkMax BACKRIGHT_DRIVE_DIRECTION_MOTOR;

    public Drive()  {
      FRONTLEFT_DRIVE_SPEED_MOTOR = new CANSparkMax(driveConstants.K_Front_Left_Drive_Motor, MotorType.kBrushless);
      FRONTRIGHT_DRIVE_SPEED_MOTOR = new CANSparkMax(driveConstants.K_Front_Right_Drive_Motor, MotorType.kBrushless);
      BACKLEFT_DRIVE_SPEED_MOTOR = new CANSparkMax(driveConstants.K_Back_Left_Drive_Motor, MotorType.kBrushless);
      BACKRIGHT_DRIVE_SPEED_MOTOR = new CANSparkMax(driveConstants.K_Back_Right_Drive_Motor, MotorType.kBrushless);

      FRONTLEFT_DRIVE_DIRECTION_MOTOR = new CANSparkMax(driveConstants.K_Front_Left_Turn_Motor, MotorType.kBrushless);
      FRONTRIGHT_DRIVE_DIRECTION_MOTOR = new CANSparkMax(driveConstants.K_Front_Right_Turn_Motor, MotorType.kBrushless);
      BACKLEFT_DRIVE_DIRECTION_MOTOR = new CANSparkMax(driveConstants.K_Back_Left_Turn_Motor, MotorType.kBrushless);
      BACKRIGHT_DRIVE_DIRECTION_MOTOR = new CANSparkMax(driveConstants.K_Back_Right_Turn_Motor, MotorType.kBrushless);

      var FrontLeft_Drive_Speed_Encoder = FRONTLEFT_DRIVE_SPEED_MOTOR.getEncoder();
      var FrontRight_Drive_Speed_Encoder = FRONTRIGHT_DRIVE_SPEED_MOTOR.getEncoder();
      var BackLeft_Drive_Speed_Encoder = BACKRIGHT_DRIVE_SPEED_MOTOR.getEncoder();
      var BackRight_Drive_Speed_Encoder = BACKRIGHT_DRIVE_SPEED_MOTOR.getEncoder();
      
      var FrontLeft_Drive_Direction_Encoder = FRONTLEFT_DRIVE_DIRECTION_MOTOR.getEncoder();
      var FrontRight_Direction_Encoder = FRONTRIGHT_DRIVE_DIRECTION_MOTOR.getEncoder();
      var BackLeft_Direction_Encoder = BACKRIGHT_DRIVE_DIRECTION_MOTOR.getEncoder();
      var BackRight_Direction_Encoder = BACKRIGHT_DRIVE_DIRECTION_MOTOR.getEncoder();

    
    }
    

    public Command move() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
              /* one-time action goes here */
            });
    }
}
