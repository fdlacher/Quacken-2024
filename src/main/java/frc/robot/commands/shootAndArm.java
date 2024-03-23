package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.shooterSubsystem;

public class shootAndArm extends Command {
    private final armSubsystem armSubsystem;
    private final shooterSubsystem shooterSubsystem;

    public shootAndArm(armSubsystem armSubsystem, shooterSubsystem shooterSubsystem){ 
        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){ 
        armSubsystem.moveArm(ScoringConstants.armMaxSpeed);
        shooterSubsystem.shoot(ScoringConstants.speakerSpeed);
    }
}
