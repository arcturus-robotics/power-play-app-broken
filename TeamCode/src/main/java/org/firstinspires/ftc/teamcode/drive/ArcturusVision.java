package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * cool wrapper for making vuforias and tfods
 */
public class ArcturusVision {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String FIRST_ELEMENT_LABEL = "Quad";
    private static final String SECOND_ELEMENT_LABEL = "Single";
    private static final String VUFORIA_LICENSE_KEY = "AagmIFb/////AAABmQeoELJRUUhBnw22N1rfAeURKsi4lO2PBquW4po2++umNgidlnmVALYdHmMXwjhAD9owXoF2zkCbWmBDEStv642zdEddYZGqPjK2pn4bDvhEeSVj4mQs3zR7mB2T94RenHo+qz8zhq4yidyNRZNYF/Y3OUTayx7H6EtYnU6kaOZi30xW6ZzrzzyP/dEG5mHV2pzBHTEu/Qe1g7RcsCG5sFDg0KAagyjxgC6X3z4/EA0tC2554q+o8S+glD7nFidnAF5e2Pti3+gAnwDN6Nl/nRGrsD0JyAVUFFk2Ii0uEorRz31VCD3C4+ib2UHN0QOZwTYgqXQ4JcH5bXJsqn1aDmSeky2smnk7xamtfXkdBm0I"; //pls dont steal :(

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public ArcturusVision(HardwareMap hardwareMap) {
        initVuforia(hardwareMap);
        initTfod(hardwareMap);
    }

    private void initVuforia(HardwareMap hardwareMap) {
        // iunstantiate vurfoaia engigne
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod(HardwareMap hardwareMap) {
        // tensor flow (tensore flow)
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, FIRST_ELEMENT_LABEL, SECOND_ELEMENT_LABEL);
    }
}
