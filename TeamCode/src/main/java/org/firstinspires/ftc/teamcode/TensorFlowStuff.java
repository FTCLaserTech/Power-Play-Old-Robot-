package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@TeleOp(name = "Tensor Flow Stuff", group = "Concept")
//@Disabled
public class TensorFlowStuff extends LinearOpMode
{
  /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
   * the following 4 detectable objects
   *  0: Ball,
   *  1: Cube,
   *  2: Duck,
   *  3: Marker (duck location tape marker)
   *
   *  Two additional model assets are available which only contain a subset of the objects:
   *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
   *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
   */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AYkCgy7/////AAABmcVEWPZVAkr+qqRZ5GKKMtplRC79gsSR0agZEVe/znTU27Ffh0FtXPIGLOSGcu+OdpREriws8ksSpiZCvHpGc8cMP5JhNkjYOk71bfFphPQeGzxAqQr+0w4bsMkf4XHP1cXHVbaVP89ifVwqpnOLSm6Z7poTfguO8PMlHnoJIL6KEdnddmgKmQclRMFlerlVjcT55VFL4YAOetN7tbBZHcC4o/zGFgXdTfQWGNug7wHPvStMAArpFZUbSMEmHMdckbXgCCGCGVZw3qYQV9D3ALkAlwvPGQo+RXckMJ3kgk6trHnzxojWVfxsuflrcyDzorAmx+qn4Ei6R+HqxkrM7mSAgV45vyVlwN5GlyF7yv8g";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.IMUInit(hardwareMap);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        TrajectorySequence moveToObj;

        Pose2d pose;
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null)
        {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).

            tfod.setZoom(2.5, 16.0/9.0);
        }

        /** Wait for the game to begin */

        boolean gp1_x_pressed = false;

        double xCoord = 0.0;
        double yCoord = 0.0;
        double moveToObjX = 0.0;
        double moveToObjY = 0.0;

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            if (tfod != null)
            {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null)
                {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    for (Recognition recognition : updatedRecognitions)
                    {

                        /*
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                         */

                        pose = drive.getPoseEstimate();

                        xCoord = findMiddleX(recognition.getLeft(), recognition.getRight());
                        yCoord = findMiddleY(recognition.getBottom(), recognition.getTop());

                        moveToObjX = (recognition.getLeft() / 10) + pose.getX();
                        moveToObjY = -((recognition.getTop() - 180) / 10) + pose.getY();

                        telemetry.addData("Current X Coordinate: ", pose.getX());
                        telemetry.addData("Current Y Coordinate: ", pose.getY());
                        telemetry.addLine();

                        telemetry.addData("X Coordinate from Camera: ", xCoord);
                        telemetry.addData("Y Coordinate from Camera: ", yCoord);
                        telemetry.addLine();

                        telemetry.addData("X Coordinate of Object: ", moveToObjX);
                        telemetry.addData("Y Coordinate of Object: ", moveToObjY);


                        if (recognition.getLabel().contentEquals("Ball") && gamepad1.x == true && gp1_x_pressed == false)
                        {
                            gp1_x_pressed = true;
                        }
                        else if (recognition.getLabel().contentEquals("Ball") && gamepad1.x == false && gp1_x_pressed == true)
                        {
                            gp1_x_pressed = false;

                            moveToObj = drive.trajectorySequenceBuilder(pose)
                                    .lineToLinearHeading(new Pose2d(moveToObjX, moveToObjY, Math.toRadians(0)))
                                    .build();
                            drive.followTrajectorySequence(moveToObj);
                        }

                        telemetry.update();
                    }

                    telemetry.update();

                }
            }
        }
    }

    private double findMiddleX(double left, double right)
    {
        return left + ((right - left) / 2);
    }

    private double findMiddleY(double bottom, double top)
    {
        return bottom + ((top - bottom) / 2);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia()
    {
        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
