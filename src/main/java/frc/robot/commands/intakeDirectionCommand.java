package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class intakeDirectionCommand extends Command{
        IntakeSubsystem intakeSubsystem;

    public intakeDirectionCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.reverseDirectionalIntake();
    }

    @Override
    public void end(boolean end) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
