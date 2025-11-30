package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

// Launch Command that doesn't worry about color, only possession
public class Launch3QuickCommand extends SequentialCommandGroup {

    public Launch3QuickCommand(IntakeSubsystem intake, LauncherSubsystem launcher, LEDSubsystem leds) {
        addCommands(
                new InstantCommand(intake::clearIntakePossessions),
                new InstantCommand(intake::readIntakePossessions),
                new WaitUntilCommand(launcher::isAtTargetSpeed),
                new ParallelCommandGroup( //launch 1st artifact
                        new InstantCommand(intake::inboundMidFront),
                        new InstantCommand(intake::inboundMidRear),
                        new InstantCommand(intake::outboundTransfer)
                ),
                new WaitUntilCommand(launcher::isNotAtTargetSpeed), //when launcher slows down, artifact is in launch
                new InstantCommand(intake::stopTransfer), //so stop transfer
                new WaitUntilCommand(launcher::isAtTargetSpeed), //to wait for it to speed back up
                new InstantCommand(intake::outboundTransfer), //then feed in artifact 2
                new WaitUntilCommand(launcher::isNotAtTargetSpeed), //when launcher slows down, artifact is in launch
                new InstantCommand(intake::stopTransfer), //so stop transfer
                new WaitUntilCommand(launcher::isAtTargetSpeed), //to wait for it to speed back up
                new InstantCommand(intake::outboundTransfer), //then feed in artifact 3
                new WaitUntilCommand(launcher::isNotAtTargetSpeed), //when launcher slows down, artifact is in launch
                new ParallelCommandGroup( //all balls launched, so stop launch sequence
                        new InstantCommand(intake::stopAll),
                        new InstantCommand(intake::stopTransfer)
                )
        );
    }
}
