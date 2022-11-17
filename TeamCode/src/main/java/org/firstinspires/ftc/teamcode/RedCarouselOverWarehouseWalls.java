package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Disabled
@Config
@Autonomous(group = "a")
public class RedCarouselOverWarehouseWalls extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.IMUInit(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        extras.initArm();
        sleep(2000);
        extras.clawClose();



        Trajectory hub = null;
        Trajectory hubBackup = null;
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        // scan for capstone
        ExtraOpModeFunctions.MarkerPosition markerPosition = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.RED);


        // Move close to carousel
        Trajectory carousel = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(
                        new Pose2d(8, 23, Math.toRadians(84)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        drive.followTrajectory(carousel);

        sleep(500);

        // Spinner on, wait, spinner off
        extras.leftWheelCCW();
        sleep(3000);
        extras.leftWheelOff();

        // Move towards the shipping hub
        Trajectory lineUpToHub = drive.trajectoryBuilder(carousel.end())
                .lineToLinearHeading(new Pose2d(40, 17, Math.toRadians(90)))
                .build();
        drive.followTrajectory(lineUpToHub);

        // Move to shipping hub - change depending on capstone location
        switch (markerPosition)
        {
            case RIGHT:

                hub = drive.trajectoryBuilder(lineUpToHub.end())
                        .lineToLinearHeading(new Pose2d(40, -7, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(hub);
                extras.armTopAuto();
                sleep(2000);
                break;
            case MIDDLE:

                hubBackup = drive.trajectoryBuilder(lineUpToHub.end())
                        .lineToLinearHeading(new Pose2d(40, 1, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(hubBackup);
                extras.armMid();
                sleep(2000);
                hub = drive.trajectoryBuilder(hubBackup.end())
                        .lineToLinearHeading(new Pose2d(40, -5, Math.toRadians(90))) // y coordinate was originally 1
                        .build();
                drive.followTrajectory(hub);
                break;
            case LEFT:
                //hubBackup = drive.trajectoryBuilder(lineUpToHub.end())
                       // .lineToLinearHeading(new Pose2d(40, 5, Math.toRadians(-91)))
                       // .build();
                TrajectorySequence hubTurn = drive.trajectorySequenceBuilder(lineUpToHub.end())
                        .turn(Math.toRadians(-180))
                        .build();
                drive.followTrajectorySequence(hubTurn);
                extras.armBottomAuto();
                hub = drive.trajectoryBuilder(hubTurn.end())
                        .lineToLinearHeading(new Pose2d(40, -11, Math.toRadians(-90)))
                        .build();
                drive.followTrajectory(hub);
                break;
        }

        // Open claw and put down arm
        sleep(500);
        extras.clawOpen();
        sleep(500);
        Trajectory back2FromHub = null;
        Trajectory lineUpToWall = null;
        switch (markerPosition) {
            case RIGHT:
            case MIDDLE:
                back2FromHub = drive.trajectoryBuilder(hub.end())
                        .lineToLinearHeading(new Pose2d(40, 9, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(back2FromHub);
                //go to wall
                lineUpToWall = drive.trajectoryBuilder(back2FromHub.end())
                        .lineToLinearHeading(new Pose2d(15, 9, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(lineUpToWall);
                extras.armBottomAuto();

                break;
            case LEFT:
                back2FromHub = drive.trajectoryBuilder(hub.end())
                        .lineToLinearHeading(new Pose2d(40, -8, Math.toRadians(-90)))
                        .build();
                drive.followTrajectory(back2FromHub);
                //Turn and  go towards the wall
                lineUpToWall = drive.trajectoryBuilder(back2FromHub.end())
                        .lineToLinearHeading(new Pose2d(20, -8, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(lineUpToWall);

        }
        sleep(1000);
        Trajectory toWarehouse = null;
        toWarehouse = drive.trajectoryBuilder(lineUpToWall.end())
                .lineToLinearHeading(new Pose2d(15, -86, Math.toRadians(90)))
                .build();
        drive.followTrajectory(toWarehouse);




        sleep(2500);

        // Line up and park


        }
    }