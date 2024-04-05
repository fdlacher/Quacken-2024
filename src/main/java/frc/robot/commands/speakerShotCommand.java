package frc.robot.commands;

import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
//Shoots with the speed necessary to score in the speaker.
public class speakerShotCommand extends Command {
    private final shooterSubsystem shooterSubsystem;

    public speakerShotCommand(shooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        System.out.println("Shooting!");
        shooterSubsystem.shootSpeaker();
    }

    @Override
    public void end(boolean end) {
        shooterSubsystem.endShoot();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}