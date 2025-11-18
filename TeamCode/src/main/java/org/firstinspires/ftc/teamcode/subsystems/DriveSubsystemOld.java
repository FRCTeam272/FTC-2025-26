package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Constants;

import java.util.function.DoubleSupplier;

public class DriveSubsystemOld extends SubsystemBasePlus {
    public MecanumDrive drive;

    public DriveSubsystemOld(HardwareMap hwMap, Pose2d startPos){

        this.drive = new MecanumDrive(hwMap,startPos);

    }


    public double getHeading(){
        return drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    }

    /**
     *
     * @param xPow- strafe/right and left movement of robot
     * @param yPow - forward and backward movement o robot
     * @param rotPow - rotational movement of robot
     * @param speed - speed at  which tp run
     */
    public void driveFieldCentric(double xPow, double yPow, double rotPow, double speed){
        if(Math.abs(xPow)<.05 && Math.abs(yPow)<.05) {drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),rotPow*speed)); return;}

        double targetTheta = Math.atan2(Math.toRadians(yPow), Math.toRadians(xPow));
        double robotTheta = getHeading();
        double diffTheta = Math.toDegrees(targetTheta)- Math.toDegrees(robotTheta);
        xPow = Math.cos(Math.toRadians(diffTheta))*speed;
        yPow = Math.sin(Math.toRadians(diffTheta))*speed;
        rotPow = rotPow*speed;
        rotPow = rotPow*speed;


        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(xPow, yPow), rotPow));
    }

    public void drive(double xPow, double yPow, double rotPow){
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(xPow, yPow),rotPow));
    }
    public void rest(){
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
    }

    public void printTelemetry(Telemetry t) {
        t.addLine("");
        t.addLine("MOTOR POWERS");
        t.addLine(String.format("     %1$s     %2$s", Constants.Util.round(drive.leftFront.getPower(),2), Constants.Util.round(drive.rightFront.getPower(),2)));
        t.addLine(String.format("     %1$s     %2$s", Constants.Util.round(drive.leftBack.getPower(),2), Constants.Util.round(drive.rightBack.getPower(),2)));
        t.addLine();
        t.addLine("IMU STUFF");
        t.addLine("IMU (RAW) " + drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        t.addLine("IMU (Modified) " + Math.toDegrees(drive.getHeading()));//modified with initial heading and
        t.addLine("Initial Heading" + drive.initialHeading);
        t.addLine();
        t.addLine("ODO");
        //t.addLine("position x" + drive.pose.position.x);
        //t.addLine("position y" + drive.pose.position.y);
        //t.addLine("odo heading" + Math.toDegrees(drive.pose.heading.log()));
        t.addLine();

        t.addLine("EXTRA");

        t.addLine("IMU PITCH: " + drive.lazyImu.get().getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        t.addLine("IMU Roll: " + drive.lazyImu.get().getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));

    }

    public CommandBase getDriveCommand(DoubleSupplier xPow, DoubleSupplier yPow, DoubleSupplier rotPower){
        return this.runEnd(()->{drive(xPow.getAsDouble(),yPow.getAsDouble(), rotPower.getAsDouble());}, ()->{
            rest();
        });
    }
    public CommandBase getDriveFieldCentric(DoubleSupplier xPow, DoubleSupplier yPow, DoubleSupplier rotPower, double speed){
        return this.runEnd(()->{driveFieldCentric(xPow.getAsDouble(),yPow.getAsDouble(), rotPower.getAsDouble(),speed);}, ()->{
            rest();
        });
    }



    //Methods that pass through Mecanum drive class
    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        return drive.actionBuilder(startPose);
    }
    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }
    public Pose2d getPose() {
        return drive.localizer.getPose();
    }


    @Override
    public void periodic() {
        updatePoseEstimate(); //Remove if causing problems!!

    }
}
