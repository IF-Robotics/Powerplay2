package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class CameraShortcut extends Hardwaremap {

    private static final String TFOD_MODEL_ASSET = "Signal3.tflite";
    private static final String[] LABELS = {
            "one",
            "three",
            "two"
    };
    private static final String VUFORIA_KEY =
            "AQVcqZ3/////AAABmYGJ564MV0Y1ig5il0AUGXlQvDeBPAf64M5zaR+NdniYRYBLaES5mbm7MGxbbFxS5xgB8/VoUNBJtCAjRX2NWcEZA0tX1gUrv90tZNYv4ebeSTwFleCZGD+4MI+i2SskLRegb80X/KB4dbMghNk+oco1Evax5eaRfySqJjs8FZKxNtSTwiSjXlcmE0D5OKLLmw0JSabGTGpwVbRhh+DvL7yQfHKHDRNvy7WoQN8e6HiFJ1d0P/YfXHoSAbsw01S5snqPy3QdJUbXkEPMCkyZZ4GKu6X1vV6ewcp05KcfT0LnRjSlCdedoSBQYvTDnoRF22iUG7sf9otuCScERuome5CBRLcCQ/81eUwo7ELsswzA";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private ElapsedTime runtime = new ElapsedTime();

    public void initCamera() {
        initVuforia();
        initTfod();
        if(tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }
    }

    public signalPosition getSignalPosition() {
        int timeout = 5;

        runtime.reset();
        signalPosition position = signalPosition.Timeout;

        while(runtime.seconds() < timeout && position == signalPosition.Timeout) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if(tfod != null) {
                if (updatedRecognitions != null) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABELS[0])) {
                            position = signalPosition.One;
                        } else if (recognition.getLabel().equals(LABELS[1])) {
                            position = signalPosition.Three;
                        } else if (recognition.getLabel().equals(LABELS[2])) {
                            position = signalPosition.Two;
                        }
                        return position;
                    }
                }
                //shouldn't need an else here. Else just means that it will stay the same
            } else {
                position = signalPosition.Error;
                return position;
            }
        }
        return position;
    }

    public enum signalPosition{
        One,
        Two,
        Three,
        Timeout,
        Error
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.50f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfodParameters.maxNumDetections = 3;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
