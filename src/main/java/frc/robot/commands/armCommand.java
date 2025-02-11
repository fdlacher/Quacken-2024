package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.armSubsystem;

public class armCommand extends Command {
    private final armSubsystem armSubsystem;
    private final double speed;
    public armCommand(armSubsystem armSubsystem, double speed){ 
        this.armSubsystem = armSubsystem;
        this.speed = speed;
        addRequirements(armSubsystem);
        
    }

    @Override
    public void initialize(){}
    

    @Override
    public void execute(){
        if (armSubsystem.getAngle() > ScoringConstants.armMaxSpeed) {
            armSubsystem.moveArm(-speed);
        }

        if(armSubsystem.getAngle() < ScoringConstants.armMinSpeed) { 
            armSubsystem.moveArm(speed);
        }
        //if arm angle is greater that "180" but less than "45" then :
            armSubsystem.moveArm(speed);
            //System.out.println("command error");
        //else set speed 0
    }
    
   
    @Override
    public void end(boolean end){
        armSubsystem.stopArm();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
