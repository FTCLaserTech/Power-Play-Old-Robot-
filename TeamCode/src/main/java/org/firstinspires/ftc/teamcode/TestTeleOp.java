package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled
@TeleOp(group = "a")
public class TestTeleOp extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        int IMUReset = 0;

        boolean gp2_a_pressed = false;
        boolean gp2_b_pressed = false;

        boolean gp2_dpad_left_pressed = false;
        boolean gp2_dpad_right_pressed = false;
        boolean gp2_dpad_up_pressed = false;
        boolean gp2_dpad_down_pressed = false;
        boolean gp2_init_arm_pressed = false;
        boolean gp2_left_stick_y = false;

        double stickForward;
        double stickSideways;
        double stickForwardRotated;
        double stickSidewaysRotated;
        double adjustedAngle;


       ExtraOpModeFunctions.MarkerPosition markerPosition = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.RED);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // extras.initArm();
        // ADD TELEMETRY TO SHOW WHEN ARM INIT IS DONE

        waitForStart();

        while (!isStopRequested())
        {
            markerPosition = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.BLUE);
            adjustedAngle = extras.adjustAngleForDriverPosition(drive.getRawExternalHeading(), ExtraOpModeFunctions.RobotStartPosition.STRAIGHT);
            stickForward = -gamepad1.left_stick_y;
            stickSideways = -gamepad1.left_stick_x;
            stickSidewaysRotated = (stickSideways * Math.cos(adjustedAngle)) - (stickForward * Math.sin(adjustedAngle));
            stickForwardRotated = (stickSideways * Math.sin(adjustedAngle)) + (stickForward * Math.cos(adjustedAngle));
            drive.setWeightedDrivePower(
                    new Pose2d(
                            stickForwardRotated,
                            stickSidewaysRotated,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            // RESET IMU
            if ((extras.localLop.gamepad1.back == true) && (extras.localLop.gamepad1.b == true))
            {
                IMUReset = 1;
            }
            else if (IMUReset == 1)
            {
                IMUReset = 0;

                // extras.localLop.RESET HEADING

            }

            // Claw servo open
            if (gamepad2.a == true)
            {
                gp2_a_pressed = true;
            }
            else if ((gamepad2.a == false) && (gp2_a_pressed))
            {
                gp2_a_pressed = false;
                extras.clawOpen();
            }

            // Claw servo close
            if (gamepad2.b == true)
            {
                gp2_b_pressed = true;
            }
            else if ((gamepad2.b == false) && (gp2_b_pressed))
            {
                gp2_b_pressed = false;
                extras.clawClose();
            }

            // hold the right bumper on controller 2 to turn on the right flywheel
            if (gamepad2.right_bumper)
            {
                extras.rightWheelCW();
            }
            else if (gamepad2.right_trigger > 0)
            {
                extras.rightWheelCCW();
            }
            else
            {
                extras.rightWheelOff();
            }

            // hold the left bumper on controller 2 to turn on the right flywheel
            if (gamepad2.left_bumper)
            {
                extras.leftWheelCCW();
            }
            else if (gamepad2.left_trigger > 0)
            {
                extras.leftWheelCW();
            }
            else
            {
                extras.leftWheelOff();
            }

            // INITIALIZE ARM
            if ((gamepad2.back == true) && (gamepad2.y == true))
            {
                gp2_init_arm_pressed = true;
            }
            else if ((gamepad2.back == false) && (gamepad2.y == false) && (gp2_init_arm_pressed == true))
            {
                gp2_init_arm_pressed = false;
                extras.initArm();
            }

            // set the arm to the preset position for the collecting level of the shipping hubs
            if (gamepad2.dpad_left == true)
            {
                gp2_dpad_left_pressed = true;
            }
            else if ((gamepad2.dpad_left == false) && (gp2_dpad_left_pressed))
            {
                gp2_dpad_left_pressed = false;
                extras.armCollect();
            }

            // set the arm to the preset position for the upper level of the shipping hubs
            if (gamepad2.dpad_up == true)
            {
                gp2_dpad_up_pressed = true;
            }
            else if ((gamepad2.dpad_up == false) && (gp2_dpad_up_pressed))
            {
                gp2_dpad_up_pressed = false;
                extras.armTopAuto();
            }

            // set the arm to the preset position for the middle level of the shipping hubs
            if (gamepad2.dpad_right == true)
            {
                gp2_dpad_right_pressed = true;
            }
            else if ((gamepad2.dpad_right == false) && (gp2_dpad_right_pressed))
            {
                gp2_dpad_right_pressed = false;
               // extras.armMid();
            }

            // set the arm to the preset position for the lower level of the shipping hubs
            if (gamepad2.dpad_down == true)
            {
                gp2_dpad_down_pressed = true;
            }
            else if ((gamepad2.dpad_down == false) && (gp2_dpad_down_pressed))
            {
                gp2_dpad_down_pressed = false;
                //extras.armBottom();
            }

            if (gamepad2.left_stick_y != 0)
            {
                gp2_left_stick_y = true;
                extras.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extras.arm.setPower(-gamepad2.left_stick_y);
            }
            else if (gp2_left_stick_y == true)
            {
                extras.arm.setPower(0);
                gp2_left_stick_y = false;
            }


            Pose2d poseEstimate = drive.getPoseEstimate();

        }
    }

}
