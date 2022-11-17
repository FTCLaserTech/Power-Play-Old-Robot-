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
public class BlueCarouselThroughWarehousePath extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.IMUInit(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        TrajectoryBook book = new TrajectoryBook(drive, extras);

        extras.clawClose();
        sleep(500);
        extras.initArm();

        Trajectory hub = null;
        Trajectory hubBackup = null;
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        // scan for capstone
        ExtraOpModeFunctions.MarkerPosition markerPosition = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.BLUE);

        book.BlueCarouselSpinDuck(drive.getPoseEstimate());
        drive.followTrajectorySequence(book.blueCarouselSpinDuck);

        // Move to shipping hub - change depending on capstone location
        switch (markerPosition)
        {
            case RIGHT:
                book.BlueRightBlockCarousel(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.blueRightBlockCarousel);
                break;
            case MIDDLE:
                book.BlueMiddleBlockCarousel(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.blueMiddleBlockCarousel);
                break;
            case LEFT:
                book.BlueLeftBlockCarousel(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.blueLeftBlockCarousel);
                break;
        }

        book.BlueParkInWarehouseCarousel(drive.getPoseEstimate());
        drive.followTrajectorySequence(book.blueParkInWarehouseCarousel);

    }
}
