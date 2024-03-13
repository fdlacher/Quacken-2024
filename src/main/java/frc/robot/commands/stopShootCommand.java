package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterSubsystem;
//del
public class stopShootCommand extends Command {
    shooterSubsystem shooterSubsystem;

    public stopShootCommand(shooterSubsystem shooterSubsystem){ 
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        shooterSubsystem.endShoot();
    }

    @Override
    public boolean isFinished(){ 
        return false;
    }
}
