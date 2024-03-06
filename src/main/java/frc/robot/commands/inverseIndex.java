package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class inverseIndex extends Command{
        private final IntakeSubsystem intakeSubsystem;

    public inverseIndex(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.reverseIntake();
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
