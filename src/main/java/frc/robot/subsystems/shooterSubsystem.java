package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringConstants;

public class shooterSubsystem extends SubsystemBase{
    private final CANSparkMax shootMotorBack;
    private final CANSparkMax shootMotorFront;
    private double ampShotSpeedsBack = ScoringConstants.ampShotSpeedsBack;
    private double ampShotSpeedsFront = ScoringConstants.ampShotSpeedsFront;


    public shooterSubsystem(){ 
        shootMotorBack = new CANSparkMax(ScoringConstants.kRightShootGoalMotor1, MotorType.kBrushless);
        shootMotorFront = new CANSparkMax(ScoringConstants.kLeftShootGoalMotor, MotorType.kBrushless);
    }
    public void shoot(){ 
        shootMotorBack.set(ScoringConstants.shootSpeed);
        shootMotorFront.set(ScoringConstants.shootSpeed);
    }
    public void shoot(double speed){ 
        shootMotorBack.set(speed);
        shootMotorFront.set(speed);
    }


    public void endShoot(){ 
        shootMotorBack.stopMotor();
        shootMotorFront.stopMotor();
    }
    public void shootAmp(){
        shootMotorBack.set(ampShotSpeedsBack);//325 //0.235
        shootMotorFront.set(ampShotSpeedsFront);//265 //0.16
    }
    
}
