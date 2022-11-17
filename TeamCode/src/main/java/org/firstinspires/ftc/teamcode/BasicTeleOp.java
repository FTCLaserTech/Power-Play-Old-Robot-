package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
// hi
/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "a")
public class BasicTeleOp extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
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
        boolean gp1a_pressed = false;
        boolean gp1b_pressed = false;
        boolean gp1x_pressed = false;
        boolean gp1y_pressed = false;
        boolean gp2x_pressed = false;
        boolean gp2y = false;
        boolean gp2_left_bumper_pressed = false;
        boolean gp2_right_trigger_pressed = false;
        boolean gp2_left_trigger_pressed = false;
        boolean ArmPositionSwitch = true;

        double stickForward;
        double stickSideways;
        double stickForwardRotated;
        double stickSidewaysRotated;
        double adjustedAngle;

        int armPosition = 0;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extras.wristClose();

        waitForStart();

        while (!isStopRequested())
        {
            // changes the speed of the robot based on bumpers that are held down
            double powerMultiplier = 1;
            if (gamepad1.right_bumper == true)
            {
                powerMultiplier = 0.6;
            }
            else if (gamepad1.left_bumper == true)
            {
                powerMultiplier = 0.4;
            }
            else
            {
                powerMultiplier = 0.75;
            }

            // this is the roadrunner driving code
            adjustedAngle = extras.adjustAngleForDriverPosition(drive.getRawExternalHeading(), ExtraOpModeFunctions.RobotStartPosition.STRAIGHT);
            stickForward = -gamepad1.left_stick_y * powerMultiplier; // lets us change the speed if buttons are held down
            stickSideways = -gamepad1.left_stick_x * powerMultiplier; // lets us change the speed if buttons are held down
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
            if ((gamepad1.back == true) && (gamepad1.b == true))
            {
                IMUReset = 1;
            }
            else if (IMUReset == 1)
            {
                IMUReset = 0;
                telemetry.addLine("IMU Resetting...");
                telemetry.update();
                drive.IMUInit(hardwareMap);
            }

            // gets the color and distance values from the color sensor
            NormalizedRGBA colors = extras.colorSensor.getNormalizedColors();

            // sets the LEDs based on distance sensor input and elapsed time (sets to flash red at endgame)
            if (getRuntime() >= 90 && getRuntime() <= 91)
            {
                extras.pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
                extras.displayPattern();
            }
            else
            {
                if (colors.alpha > 0.35)
                {
                    extras.pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    extras.displayPattern();
                }
                else
                {
                    extras.pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
                    extras.displayPattern();
                }
            }


            // switch the collection mode based on hard-coded instructions
            switch (extras.collectMode)
            {
                case CLAW:
                {
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
                    break;
                }
                case INTAKE:
                {
                    if (gamepad2.a)
                    {
                        extras.intakeIn();
                    }
                    else
                    {
                        extras.intakeOff();
                    }

                    if (gamepad2.b)
                    {
                        extras.intakeOut();
                    }
                    else
                    {
                        extras.intakeOff();
                    }
                    break;
                }
            }

            // hold the right trigger on controller 2 to turn on the right flywheel counter-clockwise
            if (gamepad2.right_bumper == true)
            {
                extras.rightWheelCCW();
            }

            // hold the left trigger on controller 2 to turn on the left flywheel counter-clockwise
            else if (gamepad2.left_bumper == true)
            {
                extras.leftWheelCCW();
            }
            else
            {
                extras.leftWheelOff();
                extras.rightWheelOff();
            }

            // press back and y on gamepad 2 to re-initialize the arm
            if ((gamepad2.back == true) && (gamepad2.y == true))
            {
                gp2_init_arm_pressed = true;
            }
            else if ((gamepad2.back == false) && (gamepad2.y == false) && (gp2_init_arm_pressed == true))
            {
                gp2_init_arm_pressed = false;
                extras.initArm();
            }

            // set the arm to the preset position for the upper level of the shipping hubs
            if (gamepad2.dpad_up == true)
            {
                gp2_dpad_up_pressed = true;
            }
            else if ((gamepad2.dpad_up == false) && (gp2_dpad_up_pressed))
            {
                gp2_dpad_up_pressed = false;
                switch (armPosition)
                {
                    case 0:
                        extras.armTopTele();
                        armPosition = 1;
                        break;

                    case 1:
                        extras.armMid();
                        armPosition = 2;
                        break;

                    case 2:
                        //extras.armBottomTeleOp();
                        break;
                }
            }

            // set the arm to the preset position for the collecting level of the shipping hubs
            if (gamepad2.dpad_left == true)
            {
                gp2_dpad_left_pressed = true;
            }
            else if ((gamepad2.dpad_left == false) && (gp2_dpad_left_pressed))
            {
                gp2_dpad_left_pressed = false;
                armPosition = 0;
                extras.armSharedHubLow();
            }

            // set the arm to the preset position for the middle level of the shipping hubs
            if (gamepad2.dpad_right == true)
            {
                gp2_dpad_right_pressed = true;
            }
            else if ((gamepad2.dpad_right == false) && (gp2_dpad_right_pressed))
            {
                gp2_dpad_right_pressed = false;
                armPosition = 2;
                extras.armSharedHubHigh();
            }

            // set the arm to the preset position for collecting game pieces
            if (gamepad2.dpad_down == true)
            {
                gp2_dpad_down_pressed = true;
            }
            else if ((gamepad2.dpad_down == false) && (gp2_dpad_down_pressed))
            {
                gp2_dpad_down_pressed = false;
                armPosition = 0;
                extras.armCollect();
            }

            // manual control for our arm's vertical motion
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

            //open grabber wrist claw thing
            if (gamepad2.x == true)
            {
                gp2x_pressed = true;
            }
            else if (gp2x_pressed == true && gamepad2.x == false)
            {
                extras.wristOpen();
                gp2x_pressed = false;
            }

            //close grabber wrist claw thing
            if (gamepad2.y == true)
            {
                gp2y = true;
            }
            else if (gp2y == true && gamepad2.y == false)
            {
                extras.wristClose();
                gp2y = false;
            }

            // controls the grabber on our robot's capstone collection arm
            if (gamepad2.right_trigger >= 0.5)
            {
                gp2_right_trigger_pressed = true;
            }
            else if (gp2_right_trigger_pressed == true && gamepad2.right_trigger <= 0.1)
            {
                gp2_right_trigger_pressed = false;

                if (ArmPositionSwitch == false)
                {
                    extras.capstoneArmDeposit();
                    ArmPositionSwitch = true;
                }
                else if (ArmPositionSwitch == true)
                {
                    extras.capstoneArmPlace();
                    ArmPositionSwitch = false;
                }
            }

            if (gamepad2.left_trigger >= 0.5)
            {
                gp2_left_trigger_pressed = true;
            }
            else if (gp2_left_trigger_pressed == true && gamepad2.right_trigger <=0.1)
            {
                extras.capstoneArmCarry();
                gp2_left_trigger_pressed = false;
                ArmPositionSwitch = true;
            }


            Pose2d poseEstimate = drive.getPoseEstimate();

            // adds telemetry that shows the robot's position and direction on the field
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData("Elapsed Time: ", getRuntime());

            // adds telemetry that shows data about the arm
            telemetry.addData("Limit Switch State: ", extras.armLimit.getValue());
            telemetry.addData("Arm Encoder Position: ", extras.arm.getCurrentPosition());
            telemetry.addData("Arm Position: ", armPosition);

            //adds telemetry for the shoulder
            telemetry.addData("Shoulder Encoder Position: ", extras.shoulder.getCurrentPosition());
            PIDFCoefficients pidf_pos = extras.shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            PIDFCoefficients pidf_vel = extras.shoulder.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine()
                    .addData("P", "%.3f", pidf_vel.p)
                    .addData("I", "%.3f", pidf_vel.i)
                    .addData("D", "%.3f", pidf_vel.d)
                    .addData("f", "%.3f", pidf_vel.f)
                    .addData("P", "%.3f", pidf_pos.p);




            // adds telemetry that shows the current encoder counts of the capstone arm DC motor
            //telemetry.addData("Shoulder encoder position", extras.shoulder.getCurrentPosition());

            // adds telemetry that shows the values for color and distance detected by the color/distance sensor
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addData("Alpha", "%.3f", colors.alpha);



            telemetry.update();
        }
    }

}
