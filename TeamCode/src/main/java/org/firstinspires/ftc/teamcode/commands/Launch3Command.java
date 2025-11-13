package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

public class Launch3Command extends SequentialCommandGroup {

    public Launch3Command(IntakeSubsystem intake, LauncherSubsystem launcher) {
        /*
        TODO
        What happens if one of the slots is not filled with an artifact?  This current code nneds to be tested, might hang!
        Write a V2 of this command that detects possession and can skip a slot if needed.
         */
        addCommands(
                new WaitUntilCommand(launcher::atSpeed), // wait until Launcher is at speed
                new ParallelCommandGroup(
                        new InstantCommand(intake::inboundMidFront), //start mid servos to inbound
                        new InstantCommand(intake::inboundMidRear) // to launch middle Artifact
                ),
                new ParallelRaceGroup(
                        new WaitUntilCommand(launcher::notAtSpeed), //wait until the wheel loses power as artifact passes thru
                        new WaitUntilCommand(intake::notMidPossession) // or there is nothing in mid slot
                ),

                new WaitUntilCommand(launcher::atSpeed), //wait again until wheel recovers
                new InstantCommand(intake::inboundRear), //turn on rear servo to feed rear artifact
                new ParallelRaceGroup(
                        new WaitUntilCommand(launcher::notAtSpeed), //wait until the wheel loses power as artifact passes thru
                        new WaitUntilCommand(intake::notRearPossession)
                ),
                new ParallelRaceGroup(
                        new WaitUntilCommand(launcher::notAtSpeed), //wait until the wheel loses power as artifact passes thru
                        new WaitUntilCommand(intake::notMidPossession) // or there is nothing in mid slot
                ),
                new WaitUntilCommand(launcher::atSpeed), //wait again until wheel recovers
                new ParallelCommandGroup(
                        new InstantCommand(intake::stopRear), //stop rear servo
                        new InstantCommand(intake::inboundFront) // and turn on front servo
                ),
                new ParallelRaceGroup(
                        new WaitUntilCommand(launcher::notAtSpeed), //wait until the wheel loses power as artifact passes thru
                        new WaitUntilCommand(intake::notFrontPossession)
                ),
                new ParallelRaceGroup(
                        new WaitUntilCommand(launcher::notAtSpeed), //wait until the wheel loses power as artifact passes thru
                        new WaitUntilCommand(intake::notMidPossession) // or there is nothing in mid slot
                ),
                new WaitUntilCommand(launcher::atSpeed), //wait again until wheel recovers
                new InstantCommand(intake::stop) //and stop all and return
        );

        addRequirements(intake, launcher);
    }
}
