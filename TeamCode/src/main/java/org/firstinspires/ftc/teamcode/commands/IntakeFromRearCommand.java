package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeFromRearCommand extends SequentialCommandGroup {

    public IntakeFromRearCommand(IntakeSubsystem intake) {

        addCommands(
                new ParallelCommandGroup( //turn on all intake servos to intake from rear
                        new InstantCommand(intake::outboundMidFront),
                        new InstantCommand(intake::inboundMidRear),
                        new InstantCommand(intake::inboundRear)
                ),
                new WaitUntilCommand(intake::frontPossession), // wait until the front sensor detects artifact
                new ParallelCommandGroup( // and turn front rear intake servos
                        new InstantCommand(intake::stopMidFront)
                ),
                new WaitUntilCommand(intake::midPossession), // wait until mid sensor detects artifact
                new WaitUntilCommand(intake::rearPossession), // and then wait again until rear sensor detects artifact
                new ParallelCommandGroup( //and stop the rest of the intake servos and return
                        new InstantCommand(intake::stopRear),
                        new InstantCommand(intake::stopMidRear)
                )
        );

        addRequirements(intake);
    }
}
