package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSubsystem extends SubsystemBase{
    private final CANSparkMax directionalIntakeMotor;
    private final CANSparkMax intakeMotor;


    public intakeSubsystem(int directionalIntakeMotorPort, int intakeMotorPort,boolean directionalIntakeMotorReversed, boolean intakeMotorReversed){
        directionalIntakeMotor = new CANSparkMax(directionalIntakeMotorPort, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(intakeMotorPort, MotorType.kBrushless);

        directionalIntakeMotor.setInverted(directionalIntakeMotorReversed);
        intakeMotor.setInverted(intakeMotorReversed);
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
        directionalIntakeMotor.set(speed);// value, should probally change
        intakeMotor.set(speed); // temp value
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