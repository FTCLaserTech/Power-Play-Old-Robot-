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
public class IntakeTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        int IMUReset = 0;

        boolean gp2_a_pressed = false;
        boolean gp2_b_pressed = false;

        extras.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (!isStopRequested())
        {

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
                }
            }
        }
    }

}
