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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ampArmSubsystem extends SubsystemBase { 
    private final CANSparkMax ampArmMotor;


    public ampArmSubsystem() { 
        ampArmMotor = new CANSparkMax(ScoringConstants.kAmpArmMotor, MotorType.kBrushless);
    }

    public void move(double speed) { 
        ampArmMotor.set(speed);
    }

    public void stop(){ 
        ampArmMotor.stopMotor();
    }
}
