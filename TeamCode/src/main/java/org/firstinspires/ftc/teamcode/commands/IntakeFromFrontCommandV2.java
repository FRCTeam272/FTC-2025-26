package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

// Intake Command that saves colors on the Rear and Mid slots during intake
public class IntakeFromFrontCommandV2 extends SequentialCommandGroup {
    public IntakeFromFrontCommandV2(IntakeSubsystem intake, LEDSubsystem leds) {
        addCommands(
                new ParallelCommandGroup( //turn on all intake servos to intake from front
                        new InstantCommand(intake::clearIntakeLoadColors),
                        new InstantCommand(intake::clearIntakePossessions),
                        new InstantCommand(intake::inboundFront),
                        new InstantCommand(intake::inboundMidFront),
                        new InstantCommand(intake::outboundMidRear),
                        new InstantCommand(intake::stopRear),
                        new InstantCommand(leds::setIntakingFront)
                ),
                new ParallelRaceGroup( //watch for artifact to go past mid (or continue if it's missed
                        new WaitUntilCommand(intake::rearPossession),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(intake::midPossession),
                                new InstantCommand(intake::setRearColor)
                        )
                ),
                new WaitUntilCommand(intake::rearPossession),
                new InstantCommand(intake::stopMidRear),
                new WaitUntilCommand(intake::midPossession),
                new ParallelCommandGroup(
                        new InstantCommand(intake::setMidColor),
                        new InstantCommand(intake::stopMidFront)
                ),
                new WaitUntilCommand(intake::frontPossession),
                new ParallelCommandGroup(
                        new InstantCommand(intake::stopAll),
                        new InstantCommand(intake::readIntakePossessions),
                        new InstantCommand(leds::setIntakingStopped)
                )
        );
    }
}
