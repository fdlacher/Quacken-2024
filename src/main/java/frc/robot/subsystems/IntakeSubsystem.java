package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax intakeMotor;

    private final CANSparkMax indexer;


    public IntakeSubsystem(){
        intakeMotor = new CANSparkMax(ScoringConstants.kIntakeMotorPort, MotorType.kBrushless);
        indexer = new CANSparkMax(ScoringConstants.kShootSetMotor,MotorType.kBrushless);

       

        intakeMotor.setInverted(ScoringConstants.intakeMotorReversed);
    }

    public void enableIntake(double speed){
        intakeMotor.set(-speed); // temp value
        indexer.set(ScoringConstants.indexerSpeed);
    }

    public void disableIntake(){
        intakeMotor.stopMotor();
        indexer.stopMotor();
    }

    public double getIntakeMotor(){
        return intakeMotor.get();
    }

    public void reverseIntake (){
        indexer.set(-ScoringConstants.indexerSpeed);
    }
    public void rightStickIntake(double rightStickPos){
        //should take in stick position and then choose wether to do either inatke or reversed index
        if(rightStickPos > 0.0){
            enableIntake(ScoringConstants.intakeSpeed);
        }
        else if(rightStickPos < 0.0){
            indexer.set(-ScoringConstants.indexerSpeed);
            intakeMotor.set(ScoringConstants.intakeSpeed);
        }
        else{
            intakeMotor.stopMotor();
            indexer.stopMotor();
        }
    }
}