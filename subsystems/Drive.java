package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.lang.AutoCloseable;

import com.revroboics.CANSparkMax;

public class Drive extends SubsystemBase {
     // Motors
     private static CANSparkMax LEFT_FRONT_DRIVE_SPEED_MOTOR;
     private static CANSparkMax LEFT_BACK_DRIVE_SPEED_MOTOR;
     private static CANSparkMax RIGHT_FRONT_DRIVE_SPEED_MOTOR;
     private static CANSparkMax RIGHT_BACK_DRIVE_SPEED_MOTOR;
 
     private static CANSparkMAx LEFT_FRONT_DRIVE_DIRECTION_MOTOR;
     private static CANSparkMax LEFT_BACK_DRIVE_DIRECTION_MOTOR;
     private static CANSparkMax RIGHT_FRONT_DRIVE_DIRECTION_MOTOR;
     private static CANSparkMax RIGHT_BACK_DRIVE_DIRECTION_MOTOR;
 
     // Encoders
     //public static Encoder LEFT_FRONT_DRIVE_DISTANCE_ENCODER;
     //public static Encoder LEFT_BACK_DRIVE_DISTANCE_ENCODER;
     //public static Encoder RIGHT_FRONT_DRIVE_DISTANCE_ENCODER;
     //public static Encoder RIGHT_BACK_DRIVE_DISTANCE_ENCODER;
     //public static MedianPIDSource DRIVE_DISTANCE_ENCODERS;
    /*
     public static Encoder LEFT_FRONT_DRIVE_DIRECTION_ENCODER;
     public static Encoder LEFT_BACK_DRIVE_DIRECTION_ENCODER;
     public static Encoder RIGHT_FRONT_DRIVE_DIRECTION_ENCODER;
     public static Encoder RIGHT_BACK_DRIVE_DIRECTION_ENCODER;
 
     // Direction encoder wrapper that scales to degrees
     //public static PIDSourceExtended LEFT_FRONT_DRIVE_DIRECTION_SCALED;
     //public static PIDSourceExtended LEFT_BACK_DRIVE_DIRECTION_SCALED;
     //public static PIDSourceExtended RIGHT_FRONT_DRIVE_DIRECTION_SCALED;
     //public static PIDSourceExtended RIGHT_BACK_DRIVE_DIRECTION_SCALED;
 
     // Gyro
     //public static AHRS DRIVE_GYRO;
    */
    public Drive()  {}
    

    public Command move() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
              /* one-time action goes here */
            });
    }
}
