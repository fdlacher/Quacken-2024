package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.shooterSubsystem;

public class autoDriveCommand extends Command{
        DriveSubsystem m_robotDrive;

        public autoDriveCommand(DriveSubsystem m_robotDrive){ 
        this.m_robotDrive = m_robotDrive;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        m_robotDrive.drive(-0.1,0.0,0.0,OIConstants.fieldRelative,true);
    }

    @Override
    public boolean isFinished(){ 
        return false;
    }
}
