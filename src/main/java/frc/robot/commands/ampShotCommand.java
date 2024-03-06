package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterSubsystem;
//shoots for the amp (not arm position)
public class ampShotCommand extends Command{
        private final shooterSubsystem shooterSubsystem;

    public ampShotCommand(shooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.shootAmp();
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
