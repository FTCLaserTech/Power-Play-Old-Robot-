package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.vuforia.Vuforia;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.Frame;
import android.graphics.Bitmap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import static java.lang.Math.PI;

/*
 * Simple mechanum drive hardware implementation for REV hardware.
 */
@Config
public class ExtraOpModeFunctions
{
    public enum RobotStartPosition {STRAIGHT, LEFT, RIGHT};
    public enum MarkerPosition{LEFT, MIDDLE, RIGHT};

    public enum CollectMode {CLAW, INTAKE};
    public enum FieldSide {RED, BLUE};
    public CollectMode collectMode = CollectMode.CLAW;

    private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;

    public Servo claw;
    public DcMotor arm;
    public DcMotor intake;
    public LinearOpMode localLop = null;

    // parts of our capstone arm
    public DcMotorEx shoulder;
    public Servo shoulder2;
    public Servo elbow;
    public Servo wrist;

    public TouchSensor armLimit;
    public NormalizedColorSensor colorSensor;
    public ColorSensor testColorSensor;

    public CRServo leftWheel;
    public CRServo rightWheel;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;

    public ExtraOpModeFunctions(HardwareMap hardwareMap, LinearOpMode lop)
    {
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        localLop = lop;

        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder"); // unused
        shoulder2 = hardwareMap.get(Servo.class, "shoulder2"); // servo
        elbow = hardwareMap.get(Servo.class, "elbow");
        wrist = hardwareMap.get(Servo.class, "wrist");

        /*
        shoulder.setTargetPositionTolerance(10);
        PIDFCoefficients pidf = new PIDFCoefficients();
        pidf.f = 40.0;
        pidf.p = 3.5;
        pidf.i = 0.1;
        pidf.d = 2.0;
        shoulder.setVelocityPIDFCoefficients(pidf.p,pidf.i,pidf.d,pidf.f);
        shoulder.setPositionPIDFCoefficients(10.0);
         */

        shoulder.setDirection(DcMotorEx.Direction.REVERSE);
        shoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setTargetPosition(0);
        //shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shoulder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftWheel = hardwareMap.get(CRServo.class, "leftWheel");
        rightWheel = hardwareMap.get(CRServo.class, "rightWheel");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armLimit = hardwareMap.get(TouchSensor.class, "armLimit");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        testColorSensor = hardwareMap.get(ColorSensor.class, "testColorSensor");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE;
        blinkinLedDriver.setPattern(pattern);
        displayPattern();

        // Configure Vuforia by creating a Parameter object, and passing it to the com.vuforia.Vuforia engine.
        VuforiaLocalizer.Parameters parameters;

        // HardwareMap hMap = new HardwareMap();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(3);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);

    }

    //public void clawOpen(){claw.setPosition(0.53);}
    public void clawOpen()
    {
        claw.setPosition(0.52);
    }

    public void clawOpenDuck() {claw.setPosition(0.42);}

    //public void clawClose(){claw.setPosition(0.69);}
    public void clawClose()
    {
        claw.setPosition(0.62);
    }

    // starts our intake motor to INTAKE game elements
    public void intakeIn()
    {
        intake.setPower(1.0);
    }

    // starts our intake motor to DEPOSIT game elements
    public void intakeOut()
    {
        intake.setPower(-1.0);
    }

    // turns off our intake motor
    public void intakeOff()
    {
        intake.setPower(0);
    }

    // initializes our arm's limit switch and capstone arm
    public void initArm()
    {
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Move up to make sure that arm is not at the bottom.
        arm.setPower(0.3);
        localLop.sleep(900);
        arm.setPower(0.0);

        localLop.telemetry.addData("Limit ", armLimit.getValue());
        localLop.telemetry.update();

        localLop.sleep(100);

        // Move down until the limit switch is triggered.
        arm.setPower(-0.1);

        while(armLimit.getValue() == 0) // CHECK IF THIS BOOLEAN STATEMENT IS CORRECT
        {
            ;
        }
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // moves arm up slightly so that it doesn't catch on floor tape
        int target = 20;

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target); // FIND POSITION for bottom of shipping hub
        arm.setPower(0.5);

        // initializes the capstone arm
        //capstoneArmCarry();
        //wristClose();
    }

    // ARM POSITIONS
    public void armCollect()
    {
        int target = 0;

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target); // FIND POSITION for arm when touching limit switch

        arm.setPower(0.8);

        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
    public void armBottomTeleOp()
    {
        int target = 3400;

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target); // FIND POSITION for bottom of shipping hub
        arm.setPower(0.8);
    }

     */

    public void armBottomAuto()
    {
        int target = 475;

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target); // FIND POSITION for bottom of shipping hub
        arm.setPower(0.8);
    }

    public void armMid()
    {
        int target = 3275;

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target); // FIND POSITION for middle of shipping hub
        arm.setPower(0.8);
    }

    public void armTopAuto()
    {
        int target = 2915;
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target); // FIND POSITION for top of shipping hub

        arm.setPower(0.8);

    }

    public void armTopTele()
    {
        int target = 2915;
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target); // FIND POSITION for top of shipping hub

        arm.setPower(0.8);

    }

    public void armSharedHubHigh()
    {
        int target = 2400;
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target); // FIND POSITION for top of shipping hub

        arm.setPower(0.8);

    }

    public void armSharedHubLow()
    {
        int target = 3555;
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target); // FIND POSITION for top of shipping hub

        arm.setPower(0.8);

    }

    // CONTROLS FLYWHEELS
    public void leftWheelCCW() { leftWheel.setPower(-1.0); }

    public void leftWheelCW() { leftWheel.setPower(1.0); }

    public void leftWheelOff() { leftWheel.setPower(0.0); }

    public void rightWheelCCW() { rightWheel.setPower(1.0); }

    public void rightWheelCW() { rightWheel.setPower(-1.0); }

    public void rightWheelOff() { rightWheel.setPower(0.0); }
    //

    // CONTROLS THE CAPSTONE ARM

    public void capstoneShoulderCarry()
    {
        shoulder2.setPosition(0.6755);
    }

    public void capstoneElbowCarry()
    {
        elbow.setPosition(0.98);
    }

    public void capstoneArmCarry()
    {
        capstoneShoulderCarry();
        localLop.sleep(750);
        capstoneElbowCarry();
    }

    public void capstoneArmCollect()
    {
        shoulder2.setPosition(0.52);
        elbow.setPosition(0.178);
    }

    public void capstoneArmDeposit()
    {
        shoulder2.setPosition(0.609);
        elbow.setPosition(0.61);
    }

    public void capstoneArmPlace()
    {
        shoulder2.setPosition(0.617);
        elbow.setPosition(0.61);
    }

    public void wristClose()
    {
        wrist.setPosition(0.2); // double check this position using servo programmer
    }

    public void wristOpen()
    {
        wrist.setPosition(0.5); // double check this position using servo programmer
    }
    //


    public double adjustAngleForDriverPosition(double angle, RobotStartPosition robotStartPosition)
    {
        switch (robotStartPosition)
        {
            case STRAIGHT:
                break;
            case LEFT:
                angle = angle - PI/2;
                if(angle < (-PI))
                    angle = angle + (PI*2);
                break;
            case RIGHT:
                angle = angle + PI/2;
                if(angle > (PI))
                    angle = angle - (PI*2);
                break;
        }
        return angle;
    }

    protected void displayPattern()
    {
        blinkinLedDriver.setPattern(pattern);
    }

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY =
            //"ASoi9IP/////AAABmSQBmuMVVUq5gVXsfaULkZgLvnW4ruzdhovJc9GL+R9Xmb5lGMIYFvYRZyV1ErJg+KWhPS+Wdq93y9tuAx78BhGnbVHVea9LnP//4Z3B/2Mqg8mg5oes4X2L8HZpPzWNmHZWTXT1RiB2fa0ISvXztQAuazZojfivJheKIx2gw+i+jfSAxlIgzf+c9IAsvnYhQwo7XjfETP2FJ2GFTo/qlQBNj65U9+TdAyWDyLSzG3JIRYWOvTYDVh7A9r+NcDoDEYiGym5MF4FumH2dnqGD77FxjZHpf4apzHpSy16f51BlALrUOQmu+/VU5ExnSPNmE5/Zscd06kW/IHaCwkSquJUHSCZjZGlEiAU5VFSGoACQ";
            "AYkCgy7/////AAABmcVEWPZVAkr+qqRZ5GKKMtplRC79gsSR0agZEVe/znTU27Ffh0FtXPIGLOSGcu+OdpREriws8ksSpiZCvHpGc8cMP5JhNkjYOk71bfFphPQeGzxAqQr+0w4bsMkf4XHP1cXHVbaVP89ifVwqpnOLSm6Z7poTfguO8PMlHnoJIL6KEdnddmgKmQclRMFlerlVjcT55VFL4YAOetN7tbBZHcC4o/zGFgXdTfQWGNug7wHPvStMAArpFZUbSMEmHMdckbXgCCGCGVZw3qYQV9D3ALkAlwvPGQo+RXckMJ3kgk6trHnzxojWVfxsuflrcyDzorAmx+qn4Ei6R+HqxkrM7mSAgV45vyVlwN5GlyF7yv8g";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    //
    //  CODE
    public MarkerPosition grabAndProcessImage(FieldSide fieldSide)
    {
        MarkerPosition markerPosition = MarkerPosition.RIGHT;
        Image imageRGB565 = null;

        int numGreen = 0;
        int numRed = 0;
        int numBlue = 0;

        //CameraDevice.getInstance().setFlashTorchMode(true);
        CameraDevice.getInstance().start();

        try
        {
            Frame frame = vuforia.getFrameQueue().take();

           // localLop.telemetry.addData("Image found ", frame.getNumImages());
            //localLop.telemetry.update();
            //linearOpMode.sleep(2000);
            for (int i = 0; i < frame.getNumImages(); ++i)
            {
                Image image = frame.getImage(i);
                //localLop.telemetry.addData("Image Num ", frame.getNumImages());
                //localLop.telemetry.update();
                //linearOpMode.sleep(2000);
                if (image.getFormat() == PIXEL_FORMAT.RGB565)
                {
                    imageRGB565 = image;
                    //localLop.telemetry.addData("Image format ", image.getFormat());
                    //localLop.telemetry.update();
                    //linearOpMode.sleep(2000);

                    break;
                }
            }

            if (imageRGB565 != null)
            {
                // grab the image
                Bitmap bm = Bitmap.createBitmap(imageRGB565.getWidth(), imageRGB565.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(imageRGB565.getPixels());
                if (fieldSide == FieldSide.RED)
                {
                    // create some variables to index the pixels
                    int xMidMin = 0;
                    int xMidMax = 0;
                    int yMidMin = 0;
                    int yMidMax = 0;


                    //int xpix = 0;
                    //int ypix = 0;

                    // Locations of where to look for marker


                    xMidMin = (int) (((2.85) * 480) / 4);
                    xMidMax = (int) (((3.4) * 480) / 4);
                    yMidMin = (int) (((2) * 640) / 5.5);
                    yMidMax = (int) (((2.7) * 640) / 5.5);
                    //ypix = ((y1MidMin+y1MidMax)/2);
                    //xpix = ((xMidMin+xMidMax)/2);


                    int pixel = 0;

                    for (int y = yMidMin; y <= yMidMax; y++)
                    {
                        for (int x = xMidMin; x <= xMidMax; x++)
                        {
                            // yellow in RGB is 0xFFFF00
                            pixel = bm.getPixel(y, x);

                            //Green
                            if ((pixel & 0x000000ff) < 0x00000000)
                            {
                                if ((pixel & 0x00ff0000) < 0x00000000)
                                {
                                    if ((pixel & 0x0000ff00) > 0x0000ff00)
                                    {
                                        numGreen++;
                                    }
                                }
                            }
                            //RED
                            if ((pixel & 0x000000ff) > 0x00000000)
                            {
                                if ((pixel & 0x00ff0000) > 0x00ff0000)
                                {
                                    if ((pixel & 0x0000ff00) > 0x00000000)
                                    {
                                        numRed++;
                                    }
                                }
                            }
                            //BLUE
                            if ((pixel & 0x000000ff) < 0x000000ff)
                            {
                                if ((pixel & 0x00ff0000) < 0x00000000)
                                {
                                    if ((pixel & 0x0000ff00) > 0x00000000)
                                    {
                                        numBlue++;
                                    }
                                }
                            }
                        }
                    }




                    if (numGreen >= 50)
                    {
                        localLop.telemetry.addData("Green", numGreen);
                        markerPosition = MarkerPosition.MIDDLE;

                    if (numRed >= 50)
                    {
                        localLop.telemetry.addData("Red", numRed);
                        markerPosition = MarkerPosition.MIDDLE;
                    }

                    if (numBlue >= 50)
                    {
                        localLop.telemetry.addData("Blue", numBlue);
                        markerPosition = MarkerPosition.MIDDLE;
                    }

                    }
                /*
                {
                    // create some variables to index the pixels
                    int xMidMinBlue = 0;
                    int xMidMaxBlue = 0;
                    int x2LeftMinBlue = 0;
                    int x2LeftMaxBlue = 0;
                    int y2LeftMinBlue = 0;
                    int y2LeftMaxBlue = 0;
                    int y1MidMinBlue = 0;
                    int y1MidMaxBlue = 0;


                    //int xpix = 0;
                    //int ypix = 0;

                    // Locations of where to look for marker

                    //Right in this case
                    xMidMinBlue = (int) (((2.3) * 480) / 4);
                    xMidMaxBlue = (int) (((3) * 480) / 4);
                    y1MidMinBlue = (int) (((1.9) * 640) / 5.5);
                    y1MidMaxBlue = (int) (((2.7) * 640) / 5.5);
                    //ypix = ((y1MidMin+y1MidMax)/2);
                    //xpix = ((xMidMin+xMidMax)/2);


                    //Mid in this case
                    x2LeftMinBlue = (int) (((0.5) * 480) / 4);
                    x2LeftMaxBlue = (int) (((1.2) * 480) / 4);
                    y2LeftMinBlue = (int) (((2) * 640) / 5.5);
                    y2LeftMaxBlue = (int) (((2.8) * 640) / 5.5);


                    int pixel = 0;

                    for (int y = y1MidMinBlue; y <= y1MidMaxBlue; y++) {
                        for (int x = xMidMinBlue; x <= xMidMaxBlue; x++) {
                            // yellow in RGB is 0xFFFF00
                            pixel = bm.getPixel(y, x);


                            if ((pixel & 0x000000ff) < 0x0000005A) {
                                if ((pixel & 0x00ff0000) < 0x005A0000) {
                                    if ((pixel & 0x0000ff00) > 0x00005A00) {
                                        numGreenMid++;
                                    }
                                }
                            }
                        }
                    }

                    for (int y = y2LeftMinBlue; y <= y2LeftMaxBlue; y++) {
                        for (int x = x2LeftMinBlue; x <= x2LeftMaxBlue; x++) {
                            // yellow in RGB is 0xFFFF00
                            pixel = bm.getPixel(y, x);


                            if ((pixel & 0x000000ff) < 0x0000005A) {
                                if ((pixel & 0x00ff0000) < 0x005A0000) {
                                    if ((pixel & 0x0000ff00) > 0x00005A00) {
                                        numGreenLeft++;
                                    }
                                }
                            }
                        }
                    }


                    if (numGreenMid >= 50) {
                        localLop.telemetry.addData("Right", numGreenRight);
                        markerPosition = MarkerPosition.RIGHT;
                    } else if (numGreenLeft >= 50) {
                        localLop.telemetry.addData("Mid", numGreenLeft);
                        markerPosition = MarkerPosition.MIDDLE;
                    } else {
                        localLop.telemetry.addData("Left", 0);
                        markerPosition = MarkerPosition.LEFT;
                    }
                }


                */

                        // yellow in RGB is 0xFFFF00
                        int color2 = bm.getPixel(0, 0);
                        //int color1 = bm.getPixel(1279, 719);



                     //localLop.telemetry.addData("C2: ", "%x", color2);
                        //localLop.telemetry.addData("Red: ", "%d", ((color2 & 0x00ff0000) >> 16));
                        //localLop.telemetry.addData("Green: ", "%d", ((color2 & 0x0000ff00) >> 8));
                        //localLop.telemetry.addData("Blue: ", "%d", ((color2 & 0x0000000ff) >> 0));
                        //localLop.telemetry.addData("C1: ", "%x", color1);
                        //localLop.telemetry.addData("Height ", bm.getHeight());
                        //localLop.telemetry.addData("Width ", bm.getWidth());

                    localLop.telemetry.update();

                        //linearOpMode.sleep(200);

                    //else
                    {
                        // nearOpMode.telemetry.addData("No image: ", brickPosition);
                        localLop.telemetry.update();
                    }
                }

                //catch(InterruptedException exc)
                //{
                //    ;
                //}
            }

            localLop.telemetry.addData("Blue_", numBlue);
            localLop.telemetry.addData("Green_", numGreen);
            localLop.telemetry.addData("Red_", numRed);
            localLop.telemetry.update();


        }
        catch(InterruptedException exc)
        {
            exc.printStackTrace();
        }

        return markerPosition;
    }
}

