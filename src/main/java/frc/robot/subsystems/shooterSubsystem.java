package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants;

public class shooterSubsystem extends SubsystemBase{
    private final CANSparkMax shootMotorBack;
    private final CANSparkMax shootMotorFront;

    private final CANSparkMax indexer;


    public shooterSubsystem(){ 
        shootMotorBack = new CANSparkMax(ScoringConstants.kRightShootGoalMotor1, MotorType.kBrushless);
        shootMotorFront = new CANSparkMax(ScoringConstants.kLeftShootGoalMotor, MotorType.kBrushless);
        indexer = new CANSparkMax(ScoringConstants.kShootSetMotor, MotorType.kBrushless);
    }
    public void shoot(){ 
        shootMotorBack.set(0.4);
        shootMotorFront.set(0.4);
        indexer.set(0.5);
    }
    public void shoot(double speed){ 
        shootMotorBack.set(speed);
        shootMotorFront.set(speed);
    }
    public void endShoot(){ 
        shootMotorBack.set(0.0);
        shootMotorFront.set(0.0);
        indexer.set(0.0);
    }
}
