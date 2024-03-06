package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax directionalIntakeMotor;
    private final CANSparkMax intakeMotor;

    private final CANSparkMax indexer;


    public IntakeSubsystem(){
        directionalIntakeMotor = new CANSparkMax(ScoringConstants.kDirectionalIntakeMotorPort, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(ScoringConstants.kIntakeMotorPort, MotorType.kBrushless);
        indexer = new CANSparkMax(ScoringConstants.kShootSetMotor,MotorType.kBrushless);

       

        directionalIntakeMotor.setInverted(ScoringConstants.directionalIntakeMotorReversed);
        intakeMotor.setInverted(ScoringConstants.intakeMotorReversed);
    }
    
    public void reverseDirectionalIntake(){
        // if statement to revese depending on wether it is already inverted or not
        if (directionalIntakeMotor.getInverted()){
        directionalIntakeMotor.setInverted(false);
        }
        else{
            directionalIntakeMotor.setInverted(true);
        }
    }

    public void enableIntake(double speed){
        directionalIntakeMotor.set(-speed);// value, should probally change
        intakeMotor.set(-speed); // temp value
        indexer.set(ScoringConstants.indexerSpeed);
    }

    public void disableIntake(){
        directionalIntakeMotor.stopMotor();
        intakeMotor.stopMotor();
        indexer.stopMotor();
    }

    public double getDirectionalIntakeMotor(){
        return directionalIntakeMotor.get();
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
            intakeMotor.set(ScoringConstants.intakeSpeed);
            directionalIntakeMotor.set(ScoringConstants.intakeSpeed);
            indexer.set(ScoringConstants.indexerSpeed);
        }
        else if(rightStickPos < 0.0){
            indexer.set(-ScoringConstants.indexerSpeed);
        }
    }
}