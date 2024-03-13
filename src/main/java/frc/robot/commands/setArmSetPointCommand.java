package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.shooterSubsystem;

public class setArmSetPointCommand extends Command{
        armSubsystem ArmSubsystem;
        double angle;

    public setArmSetPointCommand(armSubsystem ArmSubsystem,double angle){ 
        this.ArmSubsystem = ArmSubsystem;
        addRequirements(ArmSubsystem);
    }

    @Override
    public void initialize(){
        ArmSubsystem.setSetPoint(angle);
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished(){ 
        return false;
    }
}
