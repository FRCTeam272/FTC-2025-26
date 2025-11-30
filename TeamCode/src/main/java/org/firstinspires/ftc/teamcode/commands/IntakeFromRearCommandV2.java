package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

// Intake Command that saves colors on the Front and Mid slots during intake
public class IntakeFromRearCommandV2 extends SequentialCommandGroup {
    public IntakeFromRearCommandV2(IntakeSubsystem intake, LEDSubsystem leds) {
        addCommands(
                new ParallelCommandGroup( //turn on all intake servos to intake from front
                        new InstantCommand(intake::clearIntakeLoadColors),
                        new InstantCommand(intake::stopFront),
                        new InstantCommand(intake::outboundMidFront),
                        new InstantCommand(intake::inboundMidRear),
                        new InstantCommand(intake::inboundRear),
                        new InstantCommand(leds::setIntakingRear)
                ),
                new ParallelRaceGroup( //watch for artifact to go past mid (or continue if it's missed
                        new WaitUntilCommand(intake::frontPossession),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(intake::midPossession),
                                new InstantCommand(intake::setFrontColor)
                        )
                ),
                new WaitUntilCommand(intake::frontPossession),
                new InstantCommand(intake::stopMidFront),
                new WaitUntilCommand(intake::midPossession),
                new ParallelCommandGroup(
                        new InstantCommand(intake::setMidColor),
                        new InstantCommand(intake::stopMidRear)
                ),
                new WaitUntilCommand(intake::rearPossession),
                new ParallelCommandGroup(
                        new InstantCommand(intake::stopAll),
                        new InstantCommand(leds::setIntakingStopped)
                )
        );
    }
}
