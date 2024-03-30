package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ampArmSubsystem;

public class ampArmPivotBasicCommand extends Command{
    private final ampArmSubsystem ampArmSubsystem;
    private final double speed;

    public ampArmPivotBasicCommand(ampArmSubsystem ampArmSubsystem, double speed){ 
        this.ampArmSubsystem = ampArmSubsystem;
        this.speed = speed;
    }

    @Override
    public void execute(){ 
        ampArmSubsystem.move(speed);
    }
    @Override
    public void end(boolean end){ 
        ampArmSubsystem.stop();
    }


    @Override
    public boolean isFinished(){ 
        return false;
    }
}
