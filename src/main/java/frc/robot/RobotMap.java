package frc.robot;

import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;

public class RobotMap {
    /* this is constants for the robot */
    public static final class driveConstants{
    //PIDController DRIVE_CONTROL = new PIDController(0.0015, 0.001, 0, DRIVE_ENCODER, DRIVE_MOTORS);

        public static final int K_Front_Left_Drive_Motor = 10;
        public static final int K_Back_Left_Drive_Motor = 19;
        public static final int K_Front_Right_Drive_Motor = 9;
        public static final int K_Back_Right_Drive_Motor = 2;

        public static final int K_Front_Left_Turn_Motor = 11;
        public static final int K_Back_Left_Turn_Motor = 18;
        public static final int K_Front_Right_Turn_Motor = 8;
        public static final int K_Back_Right_Turn_Motor = 3;

        public static final double Front_Left_Motor_Position = 0.0;
    }
}
