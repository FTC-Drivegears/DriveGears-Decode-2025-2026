package org.firstinspires.ftc.teamcode.opmodes.tests;

public class UnusedCode {
}
//QUICKFIRE CODE
//    enum OUTTAKE {
//
//        PUSHUP1,
//        PUSHDOWN1,
//        SORT1,
//        PUSHUP2,
//        PUSHDOWN2,
//        SORT2,
//        PUSHUP3,
//        PUSHDOWN3,
//        IDLE
//    }
//    OUTTAKE outtakeState = OUTTAKE.IDLE;
//        public void quickfire () {
//            switch (outtakeState) {
//                case IDLE:
//                    break;
//
//                case PUSHUP1:
//                    hw.pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
//                    sorterSubsystem.setIsPusherUp(true);
//                    pusherTimer.reset();
//                    outtakeState = OUTTAKE.PUSHDOWN1;
//                    break;
//
//                case PUSHDOWN1:
//                    if (pusherTimer.milliseconds() >= 1000) {
//                        hw.pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
//                        sorterSubsystem.setIsPusherUp(false);
//                        pusherTimer.reset();
//                        outtakeState = OUTTAKE.SORT1;
//                    }
//                    break;
//
//                case SORT1:
//                    sorterSubsystem.outtakeToNextPos();
//                    outtakeState = OUTTAKE.PUSHUP2;
//                    sorterTimer.reset();
//                    break;
//
//                case PUSHUP2:
//                    if (sorterTimer.milliseconds() >= 1000) {
//                        hw.pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
//                        sorterSubsystem.setIsPusherUp(true);
//                        pusherTimer.reset();
//                        outtakeState = (sorterSubsystem.getCurSorterPositionIndex() > 0) ? OUTTAKE.SORT2 : OUTTAKE.IDLE;
//                    }
//                    break;
//
//                case PUSHDOWN2:
//                    if (pusherTimer.milliseconds() >= 1000) {
//                        sorterSubsystem.setIsPusherUp(false);
//                        hw.pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
//                        outtakeState = OUTTAKE.SORT2;
//                    }
//
//                    break;
//
//                case SORT2:
//                    sorterSubsystem.turnToNextPos();
//                    outtakeState = OUTTAKE.PUSHUP3;
//                    break;
//
//                case PUSHUP3:
//                    if (sorterTimer.milliseconds() >= 1000) {
//                        hw.pusher.setPosition(PusherConsts.PUSHER_UP_POSITION);
//                        sorterSubsystem.setIsPusherUp(true);
//                        pusherTimer.reset();
//                        outtakeState = OUTTAKE.PUSHDOWN3;
//                    }
//                    break;
//
//                case PUSHDOWN3:
//                    if (pusherTimer.milliseconds() >= 1000) {
//                        sorterSubsystem.setIsPusherUp(false);
//                        hw.pusher.setPosition(PusherConsts.PUSHER_DOWN_POSITION);
//                        outtakeState = OUTTAKE.IDLE;
//                    }
//                    break;
//            }
//        }


//TURRET TRACKING CODE
//            if (gamepad2.a) {
//                llDetection = false;
//                if (gamepad2.dpad_right) {
//                    llmotor.setPower(-0.5);
//                } else if (gamepad2.dpad_left) {
//                    llmotor.setPower(0.5);
//                } else {
//                    llmotor.setPower(0);
//                }
//            } else {
//                llDetection = true;
//            }
//
//            if (llDetection) {
//                LLStatus status = limelight.getStatus();
//                telemetry.addData("LL Name", status.getName());
//                telemetry.addData("CPU", "%.1f %%", status.getCpu());
//                telemetry.addData("FPS", "%d", (int) status.getFps());
//                telemetry.addData("Pipeline", "%d (%s)",
//                        status.getPipelineIndex(),
//                        status.getPipelineType()
//                );
//
//                LLResult result = limelight.getLatestResult();
//                if (result != null && result.isValid()) {
//                    double tx = result.getTx();
//                    if (tx > 2.5) {
//                        llmotor.setPower(-0.2);
//                    } else if (tx < -2.5) {
//                        llmotor.setPower(0.2);
//                    } else {
//                        llmotor.setPower(0);
//                    }
//
//                    telemetry.addData("tx", tx);
//                    telemetry.update();
//
//                } else {
//                    if (gamepad2.dpad_right) {
//                        llmotor.setPower(-0.2);
//                    } else if (gamepad2.dpad_left) {
//                        llmotor.setPower(0.2);
//                    } else {
//                        llmotor.setPower(0);
//                    }

//POTENTIAL CODE FOR TURRET ALIGNMENT DURING TELEOP
//
//            double tolerance = 0;
//            llmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            llmotor.setTargetPosition(0);
//            llmotor.setPower(1);
//            double tx = result.getTx();
//            double error = goal - tx;
//            if(gamepad1.a){
//                int tagId = result.getFiducialResults().get(0).getFiducialId();
//                if(side.equals("RED")) {
//                    if(tagId == 24){
//                        if(Math.abs(error) > tolerance){
//                            turn to aprilTag using PID
//                        }
//                    }
//                }
//                else if(side.equals("BLUE")){
//                    if(tagId == 20){
//                        if(Math.abs(error) > tolerance){
//                            turn to aprilTag using PID
//                        }
//                    }
//                }
//            }

//POTENTIAL COLOR SORTING
//            boolean right = gamepad1.dpad_right;
//            boolean left = gamepad1.dpad_left;
//            if (right || left) { // right to spin sorter to green for outtake, left to spin sorter to purple for outtake
//                if (outtakeTimer.milliseconds() > 500) {
//                    char curColor = 'g';
//                    if (left) {
//                        curColor = 'p';
//                    }
//                    sorterSubsystem.outtakeBall(curColor);
//                    outtakeTimer.reset();
//                }
//            }

//Color sorting
//            if (intaking && colorSensingTimer.milliseconds() > 500) {
//        sorterSubsystem.detectColor();
//                if (sorterSubsystem.getIsBall()) {
//        sorterSubsystem.turnToIntake('P');
//                    colorSensingTimer.reset();
//                }
//                        }


//AUTO AIM CODE
//            if (gamepad2.left_bumper && !previousAimButton) {
//                autoAimState = !autoAimState;   // toggle on *edge* of button press
//            }
//            previousAimButton = gamepad2.left_bumper;
//
//            if (autoAimState) {
//                if (LogitechVisionSubsystem.targetApril(telemetry) > 5) {
//                    mecanumCommand.pivot(0.2);
//                } else if (LogitechVisionSubsystem.targetApril(telemetry) < -5) {
//                    mecanumCommand.pivot(-0.2);
//                } else {
//                    mecanumCommand.pivot(0);
//                }
//            }

//Auto turret code
//            double curHeading = mecanumCommand.getOdoHeading();
//            double headingDelta = curHeading - initialHeading;
//            int desiredTurretPos = initialTurretPos - (int)Math.round(headingDelta * TICKS_PER_RAD);
//            turret.setTargetPosition(desiredTurretPos);
//            turret.setPower(0.4);

//Unidentified/purpose unknown
//            if (gamepad1.a && sorterSubsystem.state == SorterSubsystem.TransferState.FIRST) {
//                sorterSubsystem.state = SorterSubsystem.TransferState.PUSH_UP;
//            }

