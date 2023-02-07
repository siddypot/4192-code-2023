package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class ChangeIntakePos extends InstantCommand {
    private Intake intake;

    public ChangeIntakePos(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (intake.getIntake())
            intake.Down();
        else
            intake.Up();
    }
}