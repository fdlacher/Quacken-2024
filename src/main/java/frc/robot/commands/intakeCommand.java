package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSubsystem;

public class intakeCommand extends Command {

    private final intakeSubsystem intakeSubsystem;

    public intakeCommand(intakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.enableIntake();
    }

    @Override
    public void end(boolean end) {
        intakeSubsystem.disableIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
