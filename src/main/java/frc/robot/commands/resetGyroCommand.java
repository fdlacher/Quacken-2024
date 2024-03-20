package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class resetGyroCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    public resetGyroCommand(DriveSubsystem driveSubsystem){ 
            this.driveSubsystem = driveSubsystem;
            addRequirements(driveSubsystem);
    }  
    @Override
    public void initialize(){
        driveSubsystem.zeroHeading();
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}


