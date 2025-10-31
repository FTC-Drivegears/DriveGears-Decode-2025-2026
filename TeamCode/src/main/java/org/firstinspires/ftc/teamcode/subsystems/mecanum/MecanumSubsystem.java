package org.firstinspires.ftc.teamcode.subsystems.mecanum;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants.*;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;

public class MecanumSubsystem {
    //rf: right front/forward
    //rb: right back
    //lb: left back
    //lf: left front/forward
    //vel: velocity
    //Main: driver controlled or main program
    //Adjustment: async process controlled

    // --- Dependencies ---
    private final Hardware hw;

    // --- PID Controllers ---
    private final PIDCore globalXController;
    private final PIDCore globalYController;
    private final PIDCore globalThetaController;

    // --- Internal State: Raw Motor Outputs (Power or Velocity) ---
    public double rightFrontMotorOutput = 0;
    public double rightBackMotorOutput = 0;
    public double leftBackMotorOutput = 0;
    public double leftFrontMotorOutput = 0;

    // --- Target Wheel Velocities (radians/s) ---
    private double lfvel = 0;
    private double lbvel = 0;
    private double rfvel = 0;
    private double rbvel = 0;

    // --- Internal State: Layered Velocity Adjustments (radians/s) ---
    // Layer 0: Main driver/autonomous control input
    private double lfVelMain = 0;
    private double lbVelMain = 0;
    private double rfVelMain = 0;
    private double rbVelMain = 0;

    private PinPointOdometrySubsystem pinPointOdoSubsystem;

    // hardware is owned by test and pass down to subsystems

    private ElapsedTime elapsedTime;
    public double xFinal, yFinal, thetaFinal;
    public double velocity;


    private double ex = 0;
    private double ey = 0;
    private double etheta = 0;


    public MecanumSubsystem(Hardware hw) {
        this.hw = hw;
        this.pinPointOdoSubsystem = new PinPointOdometrySubsystem(hw);
        elapsedTime = new ElapsedTime();
        xFinal = pinPointOdoSubsystem.getX();
        yFinal = pinPointOdoSubsystem.getY();
        thetaFinal = pinPointOdoSubsystem.getHeading();
        velocity = 0;
        turnOffInternalPID();


        // initialize PID controllers
        globalXController = new PIDCore(kpx, kdx, kix);
        globalYController = new PIDCore(kpy, kdy, kiy);
        globalThetaController = new PIDCore(kptheta, kdtheta, kitheta);

        //DriveGears
//        hw.lf.setDirection(DcMotorSimple.Direction.REVERSE);
//        hw.rf.setDirection(DcMotorSimple.Direction.FORWARD);
//        hw.lb.setDirection(DcMotorSimple.Direction.REVERSE);
//        hw.rb.setDirection(DcMotorSimple.Direction.FORWARD);

        //Pr0Teens
        hw.lb.setDirection(DcMotorSimple.Direction.FORWARD);
        hw.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.rb.setDirection(DcMotorSimple.Direction.REVERSE);

        // set motor behaviour
        hw.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set motor modes
        hw.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // stop all motors
        hw.lb.setPower(0);
        hw.lf.setPower(0);
        hw.rb.setPower(0);
        hw.rf.setPower(0);
    }

    public void setConstants(double kpx, double kdx, double kix, double kpy, double kdy, double kiy, double kptheta, double kdtheta, double kitheta) {
        MecanumConstants.kpx = kpx;
        MecanumConstants.kdx = kdx;
        MecanumConstants.kix = kix;
        MecanumConstants.kpy = kpy;
        MecanumConstants.kdy = kdy;
        MecanumConstants.kiy = kiy;
        MecanumConstants.kptheta = kptheta;
        MecanumConstants.kdtheta = kdtheta;
        MecanumConstants.kitheta = kitheta;
        updatePIDConstants();
    }

    // resets all motor encoders
    public void reset() {
        hw.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // processes velocity control with no encoder feedback
    public void motorProcessNoEncoder() {
        double lfVelTemp = lfVelMain;
        double lbVelTemp = lbVelMain;
        double rfVelTemp = rfVelMain;
        double rbVelTemp = rbVelMain;

        // normalize vectors (between 0 to 1)
        double max = maxDouble(Math.abs(lfVelTemp), Math.abs(lbVelTemp), Math.abs(rfVelTemp), Math.abs(rbVelTemp));
        if (max > 1) {
            lfVelTemp /= max;
            lbVelTemp /= max;
            rfVelTemp /= max;
            rbVelTemp /= max;
        }

        lfvel = lfVelTemp;
        lbvel = lbVelTemp;
        rfvel = rfVelTemp;
        rbvel = rbVelTemp;

        // set motor powers

        setPowers(rfvel, lbvel, rbvel, lfvel);
    }

    //    named maxDouble temporarily to avoid name conflicts with local variable
    private double maxDouble(double... nums) {
        double max = -Double.MAX_VALUE;
        for (double num : nums) {
            max = Math.max(max, num);
        }
        return max;
    }

    //
    public void partialMove(double verticalVel, double horizontalVel, double rotationalVel) {
        rbVelMain = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45))) * (1.41421356237);
        rfVelMain = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) + rotationalVel * Math.sin(Math.toRadians(45))) * (1.41421356237);
        lfVelMain = (verticalVel * Math.cos(Math.toRadians(45)) + horizontalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45))) * (1.41421356237);
        lbVelMain = (-horizontalVel * Math.cos(Math.toRadians(45)) + verticalVel * Math.sin(Math.toRadians(45)) - rotationalVel * Math.sin(Math.toRadians(45))) * (1.41421356237);
    }

    public void turnOffInternalPID() {
        hw.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hw.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Basic Mecanum robot movement
    public void move( double vertical, double horizontal, double rotational){
            rightFrontMotorOutput = (-horizontal * Math.cos(Math.toRadians(45)) + vertical * Math.sin(Math.toRadians(45)) + rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);
            leftFrontMotorOutput = (vertical * Math.cos(Math.toRadians(45)) + horizontal * Math.sin(Math.toRadians(45)) - rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);
            rightBackMotorOutput = (vertical * Math.cos(Math.toRadians(45)) + horizontal * Math.sin(Math.toRadians(45)) + rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);
            leftBackMotorOutput = (-horizontal * Math.cos(Math.toRadians(45)) + vertical * Math.sin(Math.toRadians(45)) - rotational * Math.sin(Math.toRadians(45)))*(1.41421356237);

            setPowers(rightBackMotorOutput,leftBackMotorOutput,rightFrontMotorOutput,leftFrontMotorOutput);
    }

    //Update PID controllers with new constants
    public void updatePIDConstants(){
        globalXController.setConstant(kpx, kdx, kix);
        globalYController.setConstant(kpy, kdy, kiy);
        globalThetaController.setConstant(kptheta, kdtheta, kitheta);
    }

    // Output Positional Getters for PID controllers
    public double globalXControllerOutputPositional(double XsetPoint, double Xfeedback) {
        return globalXController.outputPID(XsetPoint, Xfeedback);
    }
    public double globalYControllerOutputPositional(double YsetPoint, double Yfeedback) {
        return globalYController.outputPID(YsetPoint, Yfeedback);
    }
    public double globalThetaControllerOutputPositional(double ThetasetPoint, double Thetafeedback) {
        return globalThetaController.outputPID(ThetasetPoint, Thetafeedback);
    }

    // stop all motors
    public void stop(){
            setPowers(0,0,0,0);
    }

    public void processPID() {
        ex = globalXControllerOutputPositional(xFinal, pinPointOdoSubsystem.getX());
        ey = globalYControllerOutputPositional(yFinal, pinPointOdoSubsystem.getY());
        etheta = globalThetaControllerOutputPositional(thetaFinal, pinPointOdoSubsystem.getHeading());


        double max = Math.max(Math.abs(ex), Math.abs(ey));
        if (max > velocity) {
            double scalar = velocity / max;
            ex *= scalar;
            ey *= scalar;
            etheta *= scalar;
        }

        double angle = Math.PI / 2 - pinPointOdoSubsystem.getHeading();
        double localVertical = ex * Math.cos(pinPointOdoSubsystem.getHeading()) - ey * Math.cos(angle);
        double localHorizontal = ex * Math.sin(pinPointOdoSubsystem.getHeading()) + ey * Math.sin(angle);
        partialMove(localVertical, localHorizontal, etheta);
    }


    public void resetPinPointOdometry() {
        pinPointOdoSubsystem.reset();
    }


    public void setPowers (double rightFront, double leftFront, double rightBack, double leftBack){
        hw.rf.setPower(rightFront);
        hw.lb.setPower(leftFront);
        hw.rb.setPower(rightBack);
        hw.lf.setPower(leftBack);
    }

    public boolean moveToPos(double x, double y, double theta) {
        elapsedTime.reset();
        setFinalPosition( 30, x, y, theta);
        return positionNotReachedYet();
    }

    public void setFinalPosition(double velocity, double x, double y, double theta) {
        this.xFinal = x;
        this.yFinal = y;
        this.thetaFinal = theta;
        this.velocity = velocity;

    }

    public boolean positionNotReachedYet() {
        return (isXReached() && isYReached() && isThetaReached());
    }

    public double getXDifferencePinPoint() {
        return Math.abs(this.xFinal - pinPointOdoSubsystem.getX());
    }

    public double getYDifferencePinPoint() {
        return Math.abs(this.yFinal - pinPointOdoSubsystem.getY());
    }

    public double getThetaDifferencePinPoint() {
        return Math.abs(this.thetaFinal - pinPointOdoSubsystem.getHeading());
    }

    public boolean isYReached() {
        return getYDifferencePinPoint() < 2.5;
    }

    public boolean isXReached() {
        return getXDifferencePinPoint() < 2.5;
    }

    public boolean isThetaReached() {
        return getThetaDifferencePinPoint() < 0.07;
    }

    public double getOdoX(){
        return pinPointOdoSubsystem.getX();
    }
    public double getOdoY(){
        return pinPointOdoSubsystem.getY();
    }
    public double getOdoHeading(){
        return pinPointOdoSubsystem.getHeading();
    }
    public boolean isThetaPassed(){
        return getThetaDifferencePinPoint() < 0.22;
    }

    public boolean isXPassed(){
        return getXDifferencePinPoint() < 10;
    }
    public boolean isYPassed(){
        return getYDifferencePinPoint() < 10;
    }


    //teleop

    public double fieldOrientedMove(double y, double x, double z) {
        double theta = pinPointOdoSubsystem.getHeading();
        // translate the field relative movement (joystick) into the robot relative movement
        //changed all 3 lines below
        double newX = x * Math.cos(theta) - y * Math.sin(theta);
        double newY = x * Math.sin(theta) + y * Math.cos(theta);

        // wheel power calculations
        rightFrontMotorOutput = - newY + newX - z;
        leftFrontMotorOutput = newY + newX + z;
        rightBackMotorOutput = newY + newX -  z;
        leftBackMotorOutput = - newY + newX + z;

        // normalize powers to maintain ratio while staying within the range of -1 to 1
        double largest = Math.max(
                Math.max(Math.abs(rightFrontMotorOutput), Math.abs(leftFrontMotorOutput)),
                Math.max(Math.abs(rightBackMotorOutput), Math.abs(leftBackMotorOutput)));

        if (largest > 1) {
            rightFrontMotorOutput /= largest;
            leftFrontMotorOutput /= largest;
            rightBackMotorOutput /= largest;
            leftBackMotorOutput /= largest;
        }

        // apply scaling and set motor powers
        rightFrontMotorOutput *= POWER_SCALE_FACTOR;
        leftFrontMotorOutput *= POWER_SCALE_FACTOR;
        rightBackMotorOutput *= POWER_SCALE_FACTOR;
        leftBackMotorOutput *= POWER_SCALE_FACTOR;

        setPowers(rightFrontMotorOutput,leftBackMotorOutput,rightBackMotorOutput,leftFrontMotorOutput);

        return pinPointOdoSubsystem.getHeading();
    }

    public void motorProcess() {
        processPID();
        motorProcessNoEncoder();
    }

    public void processOdometry() {
        pinPointOdoSubsystem.processOdometry();
    }
    public double getX(){
        return pinPointOdoSubsystem.getX();
    }
    public double getY(){
        return pinPointOdoSubsystem.getY();
    }
}

