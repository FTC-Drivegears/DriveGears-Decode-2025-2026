package org.firstinspires.ftc.teamcode.opmodes.tests.coloursensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Artifact;
import java.util.ArrayList;

@TeleOp(name = "colourSensor")

public class ColourAuto extends LinearOpMode {

    int red;
    int green;
    int blue;
    int alpha;
    boolean detectedColor = false;
    ArrayList<String> pattern = new ArrayList<>();
    ArrayList<Artifact> sorter = new ArrayList<Artifact>();

    @Override
    public void runOpMode() {
        ColorSensor colourSensor;
        colourSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colour1");
        colourSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colour2");

        Servo servo = hardwareMap.get(Servo.class, "servo");
        colourSensor.enableLed(true);

        servo.setPosition(0);

        pattern.add("Purple");
        pattern.add("Green");
        pattern.add("Purple");

        waitForStart();

        while (opModeIsActive()) {

            red = colourSensor.red();
            green = colourSensor.green();
            blue = colourSensor.blue();
            alpha = colourSensor.alpha();

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha", alpha);
            telemetry.addData("Amount of Artifacts", sorter.size());
            telemetry.addData("position", servo.getPosition());
            telemetry.update();

            // After intake it needs to detect 3 balls that enter the robot
            detectColour(servo);

            /*
            // After the colour is detected turn to the colour to prepare it for launch
            turnToColour(pattern.get(0), servo);
            sleep(2000);
            // add launch code

            turnToColour(pattern.get(1), servo);
            sleep(2000);
            // add launch code

            turnToColour(pattern.get(2), servo);
            sleep(2000);
            // add launch code
             */
        }
    }

    public void detectColour(Servo servo) {
        //If the sorter is full it stops
        if (sorter.size() == 3) {
            return;
        }

        // Purple ball is detected
        if (blue > green && alpha < 700) {
            if (detectedColor == false) {
                detectedColor = true;
                sorter.add(new Artifact("Purple", servo.getPosition()));
                telemetry.addLine("Purple Detected");
                turnSorter(servo);
            } //green detected
        } else if (green - blue > 20 && alpha < 110 && alpha > 70) {
            if (detectedColor == false) {
                detectedColor = true;
                sorter.add(new Artifact("Green", servo.getPosition()));
                telemetry.addLine("Green Detected");
                turnSorter(servo);
            }

        }else {
            detectedColor = false;
        }
        telemetry.update();
    }

    public void turnSorter(Servo servo) {
        //If the sorter is full it stops
        //if (sorter.size() == 3) {
            if (sorter.size() == 3) {
            return;
        }

        if (servo.getPosition() != 1) {
            //make sure the servo doesn't break
            servo.setPosition(sorter.size() * 0.45 );
        }
    }

    public void turnToColour(String color, Servo servo) {
        double pos = 0;
        if (color.equals("Green")) {
            //gets position of the first green it finds
            pos = sorter.get(sorter.indexOf("Green")).getPosition();
        }

        if (color.equals("Purple")) {
            //gets position of the first purple it finds
            pos = sorter.get(sorter.indexOf("Purple")).getPosition();
        }

        //move servo
        servo.setPosition(pos);
    }

//    public void quickFire(Servo servo) {
//        servo.setPosition(sorter.get(0).getPosition());
//        sleep(800);
//        //launch
//        sorter.remove(0);
//
//        servo.setPosition(sorter.get(0).getPosition());
//        sleep(500);
//        //launch
//        sorter.remove(0);
//
//        servo.setPosition(sorter.get(0).getPosition());
//        sleep(500);
//        //launch
//        sorter.remove(0);
//    }
}