package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooterSubsystem extends SubsystemBase{
    private final CANSparkMax shootSetMotor1;

    private final CANSparkMax shootGoalMotor1;
    private final CANSparkMax shootGoalMotor2;


    public shooterSubsystem(){ 
        shootSetMotor1 = new CANSparkMax(3, MotorType.kBrushless);
        shootGoalMotor1 = new CANSparkMax(5, MotorType.kBrushless);
        shootGoalMotor2 = new CANSparkMax(6, MotorType.kBrushless);
    }
    public void shoot(){ 
        shootSetMotor1.set(0.5);
        shootGoalMotor1.set(0.5);
        shootGoalMotor2.set(0.5);
    }
    public void endShoot(){ 
        shootSetMotor1.set(0.0);
        shootGoalMotor1.set(0.0);
        shootGoalMotor2.set(0.0);
    }
}
