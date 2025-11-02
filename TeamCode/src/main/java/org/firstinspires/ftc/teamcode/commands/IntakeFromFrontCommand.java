package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeFromFrontCommand extends SequentialCommandGroup {

    public IntakeFromFrontCommand(IntakeSubsystem intake) {

        addCommands(
                new ParallelCommandGroup( //turn on all intake servos to intake from front
                        new InstantCommand(intake::inboundFront),
                        new InstantCommand(intake::inboundMidFront),
                        new InstantCommand(intake::outboundMidRear),
                        new InstantCommand(intake::outboundRear)
                ),
                new WaitUntilCommand(intake::rearPossession), // wait until the rear sensor detects artifact
                new ParallelCommandGroup( // and turn off rear intake servos
                        new InstantCommand(intake::stopMidRear),
                        new InstantCommand(intake::stopRear)
                ),
                new WaitUntilCommand(intake::midPossession), // wait until mid sensor detects artifact
                new WaitUntilCommand(intake::frontPossession), // and then wait again until front sensor detects artifact
                new ParallelCommandGroup( //and stop the rest of the intake servos and return
                        new InstantCommand(intake::stopFront),
                        new InstantCommand(intake::stopMidFront)
                )
        );

        addRequirements(intake);
    }
}
