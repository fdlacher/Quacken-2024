package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringConstants;
public class armSubsystem extends SubsystemBase{
    private final CANSparkMax leftArmMotor;
    private final CANSparkMax rightArmMotor;

    private final SparkAbsoluteEncoder armEncoder;
    //we need an absolute encoder
    public armSubsystem(){ 
        leftArmMotor = new CANSparkMax(ScoringConstants.kleftArmMotor, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(ScoringConstants.kRightArmMotor, MotorType.kBrushless);
        
        armEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    }

    public void moveArm(double speed){ 
        leftArmMotor.set(speed);
        rightArmMotor.set(-speed);
    }


    public void stopArm(){ 
        leftArmMotor.stopMotor();
        rightArmMotor.stopMotor();
    }

    public double getAngle(){ 
        return armEncoder.getPosition();
    }

}
