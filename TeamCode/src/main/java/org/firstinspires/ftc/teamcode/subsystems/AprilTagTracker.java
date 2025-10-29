package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous
public class AprilTagTracker extends OpMode {
    private Hardware hw;
    private CRServo turret;
    private AnalogInput encoder;

    @Override
    public void init() {
        hw = Hardware.getInstance(hardwareMap);
        turret = hw.turret;
        encoder = hw.encoder;
    }

    @Override
        public void loop() {
            turret.setPower(1);
            processTelemetry();
        }

    public double get_position() {
        double position = encoder.getVoltage() / 3.2 * 360;
        return (position / 3.2) * 360.0;
    }

    public void processTelemetry() {
        telemetry.addData("position: ", get_position());
        telemetry.update();
    }
}
