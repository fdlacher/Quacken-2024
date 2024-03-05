package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants;

public class shooterSubsystem extends SubsystemBase{
    private final CANSparkMax shootMotorBack;
    private final CANSparkMax shootMotorFront;


    public shooterSubsystem(){ 
        shootMotorBack = new CANSparkMax(ScoringConstants.kRightShootGoalMotor1, MotorType.kBrushless);
        shootMotorFront = new CANSparkMax(ScoringConstants.kLeftShootGoalMotor, MotorType.kBrushless);
    }
    public void shoot(){ 
        shootMotorBack.set(0.4);
        shootMotorFront.set(0.4);
    }
    public void endShoot(){ 
        shootMotorBack.set(0.0);
        shootMotorFront.set(0.0);
    }
}
