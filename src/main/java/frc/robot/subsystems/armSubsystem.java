package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringConstants;
public class armSubsystem extends SubsystemBase{
    private final CANSparkMax leftArmMotor;
    private final CANSparkMax rightArmMotor;

    private final SparkAbsoluteEncoder armEncoder;
    double setPoint =0.0;
    double currentAngle;
    //shuffleboard
    private PIDController armPID = new PIDController(0.2,0.5,.5);//Tune these values!!

    private GenericEntry d_pidOutput = Shuffleboard.getTab("Intake").add("PID output", 0).getEntry();
    private GenericEntry d_currentAngle = Shuffleboard.getTab("Intake").add("Current Angle", 0).getEntry();
    
    //we need an absolute encoder
    public armSubsystem(){ 
        leftArmMotor = new CANSparkMax(ScoringConstants.kleftArmMotor, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(ScoringConstants.kRightArmMotor, MotorType.kBrushless);
        
        armEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
        Shuffleboard.getTab("Intake").add("Arm PID", armPID);
    }

    public void moveArm(double speed){ 
        System.out.println("Moving arm" + speed);
        leftArmMotor.set(speed);
        rightArmMotor.set(-speed);
        
    }
    public void moveArm(double speed, double leftStickPos){
        
        if(leftStickPos > 0.0){
            leftArmMotor.set(speed);
            rightArmMotor.set(-speed);
        }
        else if(leftStickPos < 0.0){
            leftArmMotor.set(-speed);
            rightArmMotor.set(speed);
        }
        else{
            leftArmMotor.stopMotor();
            rightArmMotor.stopMotor();
        }
    }


    public void stopArm(){ 
        leftArmMotor.stopMotor();
        rightArmMotor.stopMotor();
    }

    public double getAngle(){ 
        return armEncoder.getPosition();
    }
    public void moveArmSpecific(double endAngle){
        System.out.println("testing");
        var currentANgle = getAngle();
        d_currentAngle.setDouble(currentANgle);
        double speed = -armPID.calculate(currentANgle);
        d_pidOutput.setDouble(speed);
        System.out.println(speed);
        speed = MathUtil.clamp(speed,-ScoringConstants.armMinSpeed, -ScoringConstants.armMaxSpeed);
        moveArm(speed);
    }
    public void goToSetPoint(){
        currentAngle = armEncoder.getPosition();

        var PIDoutput = armPID.calculate(currentAngle,setPoint);
        d_currentAngle.setDouble(currentAngle);
        d_pidOutput.setDouble(PIDoutput);
        moveArm(MathUtil.clamp(PIDoutput,-0.05,0.1));
        /* 
        if(currentAngle > .17 && setPoint == 0.0){
            moveArm(MathUtil.clamp(PIDoutput,0.05,0.1));
        }
        else{
            moveArm(MathUtil.clamp(PIDoutput,-0.05,0.1));
        }
        */
        /*
        if(){ // if higher than highest point?
            //need clamp
            moveArm(PIDoutput);
        }
        else if(){ // less then lowest point
            //need clamp
            moveArm(PIDoutput);
        }
        else{
            stopArm();
        }
        */
    }
    public void setSetPoint(double angle){
        setPoint = angle;
    }

    public void runArm(){
        this.run(()->goToSetPoint());//not sure if that works
    }
}
