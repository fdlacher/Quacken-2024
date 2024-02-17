package frc.robot;

import edu.wpi.first.math.util.Units;

public class RobotMap {
    /* this is constants for the robot */
    public static final class DriveConstants{
    //PIDController DRIVE_CONTROL = new PIDController(0.0015, 0.001, 0, DRIVE_ENCODER, DRIVE_MOTORS);

        public static final int K_Front_Left_Drive_Motor_Port = 10;
        public static final int K_Back_Left_Drive_Motor_Port = 19;
        public static final int K_Front_Right_Drive_Motor_Port = 9;
        public static final int K_Back_Right_Drive_Motor_Port = 2;

        public static final int K_Front_Left_Turn_Motor_Port = 11;
        public static final int K_Back_Left_Turn_Motor_Port = 18;
        public static final int K_Front_Right_Turn_Motor_Port = 8;
        public static final int K_Back_Right_Turn_Motor_Port = 3;

        public static final boolean K_Front_Left_Drive_Encoder_Reversed = false;
        public static final boolean K_Front_Right_Drive_Encoder_Reversed = false;
        public static final boolean K_Back_Left_Drive_Encoder_Reversed = false;
        public static final boolean K_Back_Right_Drive_Encoder_Reversed = false;

        public static final boolean K_Front_Left_Turn_Encoder_Reversed = false;
        public static final boolean K_Front_Right_Turn_Encoder_Reversed = false;
        public static final boolean K_Back_Left_Turn_Encoder_Reversed = false;
        public static final boolean K_Back_Right_Turn_Encoder_Reversed = false;

        public static final double Front_Left_Motor_Position = 0.0;
    }
    public static final class ModuleConstants{
        public static final double KWheelDiameterMeters = Units.inchesToMeters(0/*input wheel diameter*/);
        public static final double kDriveMotorGearRatio = 1/5/*gear ratio */;
        public static final double KTurningMotorGearRatio = 1/5/*gear ratio */;
        public static final double KDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * KWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = KTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MetersPerSec = KDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2Meter = kTurningEncoderRot2Rad/ 60;
        public static final double KPTurning = 0.5;

    }
}
