package frc.robot.commands;

import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class shootCommand extends Command {
    private final shooterSubsystem shooterSubsystem;

    public shootCommand(shooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        System.out.println("Shooting!");
        shooterSubsystem.shoot(ScoringConstants.shootSpeed);
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