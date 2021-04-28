package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class RingIdentification {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AZvVY5H/////AAABmVIJC2J+Bk8dmED6q+xzKkUsIvfAZMxMAHooL0K1pItNOLCvp0Mq1vBnpcHiPjwlLYt5OOVpnoyE72D5Ku8iR1ahgCo9Le6ymYWRR/No45AHVGMbfpnYxmMchxYfGn/otWPMzvohNI2lyeJwmumFgHIO9nAJ7YH86HptFVWynm16S2ENaAUfBOeKxqPABdc7i8PCe7r/PMeY2gVSAp9/53S3Gi9PGvv4f2wCtlSQChSAsbdglIuy91gLnHHzR0w1itEjtRvSnronkqr/2P9xNfWcp8tN7duysWh6eQlUhAWerhD05mvmBorePfpmCfS2sWn4FTMdzR3vZZxypY1NgOx7OvYSEhPaapYcX8ydsBOb";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    //Target zone variables to keep hold of the number of rings - 1 (Target A),2 (Target B), 3 (Target C.
    private int targetZone = 0;

    /**
     * `Initialize the Vuforia localization engine.
     */

    public void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");
        parameters.useExtendedTracking = true;
        parameters.cameraMonitorFeedback= VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.useObjectTracker = true;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }


    public void activateTfod() {
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0 / 9.0);
        }
    }

    /**
     * Ring Identification.
     */
    public int identifyRings(Telemetry telemetry) {

             // List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0) {
                    // empty list.  no objects recognized.
                    telemetry.addData("TFOD", "No items detected.");
                    telemetry.addData("Target Zone", "A");
                    this.targetZone = 1;
                    return this.targetZone;
                } else {
                    // list is not empty.
                    // step through the list of recognitions and display boundary info.
                    //  int i = 0;
                    for (int i=0; i < updatedRecognitions.size(); i++) {
                        Recognition recognition = updatedRecognitions.get(i);
                        //  for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        // check label to see which target zone to go after.
                        if (recognition.getLabel().equals("Single")) {
                            telemetry.addData("Target Zone", "B");
                            this.targetZone = 2;
                            return this.targetZone;

                        } else if (recognition.getLabel().equals("Quad")) {
                            telemetry.addData("Target Zone", "C");
                            this.targetZone = 3;
                            return this.targetZone;
                        } else {
                            telemetry.addData("Target Zone", "UNKNOWN");
                        }
                    }

                }

                telemetry.update();

            }
        return this.targetZone;
    }


    public void deactivateTfod() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

}
