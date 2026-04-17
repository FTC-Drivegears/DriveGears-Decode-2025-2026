package org.firstinspires.ftc.teamcode.opmodes.tests.vision;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.teamcode.Hardware;
import java.util.List;

public class LimelightVision {
    private Hardware hw;
    private Limelight3A limelight;
    public String OBELISK = "UNKNOWN";

    public LimelightVision(Hardware hw) {
        this.hw = hw;
        this.limelight = hw.limelight;
    }

    public String pattern() {
//        limelight.pipelineSwitch(0); // obelisk pipeline
//        limelight.start();

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return "UNKNOWN";
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            return "UNKNOWN";
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            int id = fiducial.getFiducialId();
            if (id == 21) {
                OBELISK = "GPP";
            } else if (id == 22) {
                OBELISK = "PGP";
            } else if (id == 23) {
                OBELISK = "PPG";
            }
        }
        return OBELISK;
    }
}

