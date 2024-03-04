package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringConstants;
public class armSubsystem extends SubsystemBase{
    private final CANSparkMax leftArmMotor;
    private final CANSparkMax rightArmMotor;
    //we need an absolute encoder
    public armSubsystem(){ 
        leftArmMotor = new CANSparkMax(ScoringConstants.kleftArmMotor, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(ScoringConstants.kRightArmMotor, MotorType.kBrushless);
    }

    public void moveArm(double speed){ 
        leftArmMotor.set(speed);
        rightArmMotor.set(-speed);
    }

    public void stopArm(){ 
        leftArmMotor.set(0.0);
        rightArmMotor.set(0.0);
    }


}
