
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.armSubsystem;

public class pivotArmSpecfic extends Command {
    private final armSubsystem armSubsystem;
    private final boolean continueArm;
    private Supplier<Double> endAngle;

    private PIDController armPID = new PIDController(0.03,0,0);//Tune these values!!

    public pivotArmSpecfic(armSubsystem armSubsystem, double endAngle){ 
        this(armSubsystem, () -> endAngle, false);
    }

    public pivotArmSpecfic(armSubsystem armSubsystem, double endAngle, boolean continueArm){ 
        this(armSubsystem, () -> endAngle, continueArm);
    }
        
 
    public pivotArmSpecfic(
        armSubsystem armSubsystem, 
        Supplier<Double> endAngle, 
        boolean continueArm){
    this.armSubsystem = armSubsystem;
    this.continueArm = continueArm;
    addRequirements(armSubsystem);
    }


 
  

    @Override
    public void initialize(){
        armPID.setSetpoint(endAngle.get());
        armPID.setTolerance(2);//Tune
    }

    @Override
    public void execute(){
        double speed = armPID.calculate(armSubsystem.getAngle());
        speed = MathUtil.clamp(speed,-ScoringConstants.armMinSpeed, -ScoringConstants.armMaxSpeed);
        armSubsystem.moveArm(speed);
    }
    @Override
    public void end(boolean interrupt){ 
        armSubsystem.stopArm();
    }
    @Override
    public boolean isFinished() { 
        if(continueArm){ 
            return false;
        }

        return armPID.atSetpoint();
    }
}
