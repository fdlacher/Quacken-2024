package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax directionalIntakeMotor;
    private final CANSparkMax intakeMotor;


    public IntakeSubsystem(){
        directionalIntakeMotor = new CANSparkMax(ScoringConstants.kDirectionalIntakeMotorPort, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(ScoringConstants.kIntakeMotorPort, MotorType.kBrushless);

       

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
    }

    public void disableIntake(){
        directionalIntakeMotor.stopMotor();
        intakeMotor.stopMotor();
    }

    public double getDirectionalIntakeMotor(){
        return directionalIntakeMotor.get();
    }

    public double getIntakeMotor(){
        return intakeMotor.get();
    }
}