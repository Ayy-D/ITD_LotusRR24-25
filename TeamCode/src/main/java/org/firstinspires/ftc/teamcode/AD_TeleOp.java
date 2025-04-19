package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name="AD_TeleOp")

public class AD_TeleOp extends LinearOpMode{
    String placeholder = "----";


    //Directionally named motors based on view from opposite of back odo pod side

    //Control Hub Motors
    private DcMotorEx rightFront = null;
    private DcMotorEx leftFront = null;
    private DcMotorEx rightBack = null;
    private DcMotorEx leftBack= null;

    //Control Hub Servos
    private Servo inR = null; // SPM
    private Servo inB = null; // SPM
    private Servo inUD = null;
    private CRServo inWR = null; // SPM
    private CRServo inWL = null; // SPM


    //Expansion Hub Motors

    private DcMotorEx sL = null;
    private DcMotorEx sR = null;

    //Expansion Hub Servos
    private Servo scArm = null; //SPM
    private Servo scUD = null; //SPM
    private Servo scC = null;

    //Sensors & Cameras

    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        //drivetrain power vars
        double  drive = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn = 0;        // Desired turning power/speed (-1 to +1)

        //drivetrain motors
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        //rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        //LS Motors
        sL = hardwareMap.get(DcMotorEx.class, "slideL");
        sL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //sL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sR = hardwareMap.get(DcMotorEx.class, "slideR");
        sR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //sR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sR.setDirection(DcMotorEx.Direction.REVERSE);



        //Control Hub Servos (Intake)
        inR = hardwareMap.get(Servo.class, "inRed"); // 0.3 for collapse, 0.7 full out
        inB = hardwareMap.get(Servo.class, "inBlue"); // same as inR
        inB.setDirection(Servo.Direction.REVERSE);
        inUD = hardwareMap.get(Servo.class, "inUD"); // 0.2 full down, 0.9 max, FIND INTAKING AND TRANSFER POS
        inWR = hardwareMap.get(CRServo.class, "inWRight"); // 1.0 outtake, -1.0 intake
        inWL = hardwareMap.get(CRServo.class, "inWLeft"); // -1.0 outtake, 1.0 intake

        //Sensors + Cameras

        //Expansion Hub Servos
        scArm = hardwareMap.get(Servo.class, "scArmL"); //0.95 goes toward intake, 0 goes outward from robot
        scUD = hardwareMap.get(Servo.class, "scUD"); //1 is the position for depositing an element, 0.8 for intake, <0.8 to keep it up
        scUD.setDirection(Servo.Direction.REVERSE);
        scC = hardwareMap.get(Servo.class, "scClaw"); //0.27 close, 0.8 open

        //Sample-Specimen Cycle
        int cycleCase = 0; // 0 - Sample, 1 - Specimen, 2 - Individual System Testing

        //Intake/Scoring Trigger Cycle Variables
        int inCurrCase = 0;
        boolean inLastButtonStateR = false;
        boolean inLastButtonStateL = false;

        // Linear Slide Cycle Variables
        int scCurrCase = 0;
        boolean scArmastButtonState = false;

        //Linear Slide Reset Counter
        boolean hasResetEncoders = false;

        //Scoring Arm Counter - SAMPLE AUTOMATION ONLY
        int triangleCounter = 0;

        //Specimen Switch Counter
        int specCount = 0;

        waitForStart();
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        if (isStopRequested()) return;

        scC.setPosition(0.3);


        while (opModeIsActive())
        {

            drive  = gamepad1.left_stick_y  / (1.25 + gamepad1.right_trigger * 3);  // Reduce drive rate to 27-80%.
            strafe = gamepad1.left_stick_x  / (1.25  + gamepad1.right_trigger * 3);  // Reduce strafe rate to 27-80%.
            turn   = -gamepad1.right_stick_x / (1.5 + gamepad1.right_trigger * 3);  // turn rate 22-67%.
            moveRobot(drive, strafe, turn);


            boolean xButtonState = gamepad2.cross; // linear slides++
            boolean rightBumperState = gamepad1.right_bumper; // intaking++
            boolean leftBumperState = gamepad1.left_bumper; // intaking--


            if(gamepad2.right_trigger > 0.75){ // Switch to SPECIMEN Automation
                cycleCase = 1;
                inCurrCase = 0;
                scCurrCase = 0;
                timer.reset();

            }
            if(gamepad2.left_trigger > 0.75){ // Switch to SAMPLE Automation
                cycleCase = 0;
                inCurrCase = 0;
                scCurrCase = 0;
                specCount = 0;
            }
            if (((sL.getCurrent(CurrentUnit.AMPS) > 4 || sR.getCurrent(CurrentUnit.AMPS ) > 4) || gamepad2.dpad_down) && scCurrCase == 0) {
                resetSlideEncoders(); // Call the reset function
            }


            //Sample Automation
            if(cycleCase == 0){
                int numCycles = 7;
                if(rightBumperState && !inLastButtonStateR){  inCurrCase = (inCurrCase + 1) % (numCycles + 1);  }
                inLastButtonStateR = rightBumperState;

                if(leftBumperState && !inLastButtonStateL){   inCurrCase = (inCurrCase - 1 + (numCycles + 1)) % (numCycles + 1);   }
                inLastButtonStateL = leftBumperState;

                switch(inCurrCase){
                    case 0: // full collapsed
                        inR.setPosition(0.32);
                        inB.setPosition(0.32);
                        if(scCurrCase == 0 && (sL.getCurrentPosition() < 100 || sR.getCurrentPosition() < 100)){
                            inUD.setPosition(0.67);
                        }
                        inWR.setPower(0);
                        inWL.setPower(-0);
                        break;

                    case 1: //prep intake pos
                        inR.setPosition(0.35);
                        inB.setPosition(0.35);
                        inUD.setPosition(0.36);

                        inWR.setPower(-1);
                        inWL.setPower(1);
                        break;

                    case 2: //intaking until distance find element - push outminor/major switch
                        timer.reset();
                        inUD.setPosition(0.28);
                        inWR.setPower(-1);
                        inWL.setPower(1);

                        if(gamepad1.cross){
                            inR.setPosition(Range.clip(0.4, 0.7, inR.getPosition() + .001));
                            inB.setPosition(Range.clip(0.4, 0.7, inB.getPosition() + .001));
                        }
                        else{
                            inR.setPosition(Range.clip(0.35, .4, inR.getPosition() + .01));
                            inB.setPosition(Range.clip(0.35, 0.4, inB.getPosition() + .01));
                        }

                        if(gamepad1.triangle){
                            inWR.setPower(1);
                            inWL.setPower(-1);
                        }

                        //if(distance < 5){
                        //  inCurrCase = 3;
                        //}

                        break;


                    case 3: //prep transfer
                        inR.setPosition(0.5);
                        inB.setPosition(0.5);
                        inUD.setPosition(0.65);

                        inWR.setPower(0);
                        inWL.setPower(0);
                        break;

                    case 4: // transfer collapse - partial
                        timer.reset();
                        inR.setPosition(0.4);
                        inB.setPosition(0.4);
                        inUD.setPosition(0.9);
                        inCurrCase = 5;
                        break;

                    case 5: // outtake to bucket until distance senses nothing in intake
                        if(timer.milliseconds() > 500){
                            inWR.setPower(1);
                            inWL.setPower(-1);

                            //if(distance > 5){
                            // inCurrCase = 6;
                            //}
                        }
                        break;

                    case 6:
                        timer.reset();
                        inR.setPosition(0.41);
                        inB.setPosition(0.41);
                        inUD.setPosition(0.67);
                        inCurrCase = 7;
                        break;

                    case 7:
                        if(timer.milliseconds() > 250){
                            inCurrCase = 0;
                        }
                        break;

                }

                if(inCurrCase == 0){
                    if((gamepad2.triangle || gamepad1.triangle) && scCurrCase > 1){
                        scUD.setPosition(0.7);
                        triangleCounter = 1;
                    }
                    if(xButtonState && !scArmastButtonState && triangleCounter != 1){
                        scCurrCase = (scCurrCase + 1) % 5;

                    }
                    if(xButtonState && !scArmastButtonState && triangleCounter == 1){
                        scCurrCase = 0;
                    }

                    scArmastButtonState = xButtonState;


                    switch(scCurrCase){
                        case 0:
                            timer.reset();

                            sL.setPower(0.9);
                            sL.setTargetPosition(20);
                            sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            sR.setPower(0.9);
                            sR.setTargetPosition(20);
                            sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            scArm.setPosition(1);
                            scUD.setPosition(0.67);
                            triangleCounter = 0;
                            break;

                        case 1:
                            timer.reset();
                            scArm.setPosition(0.48);
                            inUD.setPosition(0.5);

                            scCurrCase = 2;
                            break;

                        case 2:
                            if(timer.milliseconds() > 400) {
                                scUD.setPosition(0.4);
                                scCurrCase = 3;
                            }

                            break;

                        case 3:
                            sL.setPower(0.9);
                            sL.setTargetPosition(700);
                            sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            sR.setPower(0.9);
                            sR.setTargetPosition(700);
                            sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            break;

                        case 4:
                            scArm.setPosition(0.48);
                            sL.setPower(0.9);
                            sL.setTargetPosition(1850);
                            sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            sR.setPower(0.9);
                            sR.setTargetPosition(1850);
                            sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            break;
                    }

                }
            }

            //Specimen Automation
            if(cycleCase == 1){
                if(rightBumperState && !inLastButtonStateR){  inCurrCase = (inCurrCase + 1) % 6;  }
                inLastButtonStateR = rightBumperState;

                if(leftBumperState && !inLastButtonStateL){   inCurrCase = (inCurrCase - 1 + 6) % 6;   }
                inLastButtonStateL = leftBumperState;

                switch(inCurrCase){
                    case 0: // full collapsed - grab from wall
                        break;

                }
            }


            telemetry.addData("-----", placeholder);

            telemetry.addData("sR Position", sR.getCurrentPosition());
            telemetry.addData("sR Current Draw", sR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("sL Position", sL.getCurrentPosition());
            telemetry.addData("sL Current Draw", sL.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("-----", placeholder);

            if(cycleCase == 0){
                telemetry.addData("Cycle Case", "___SAMPLE___");
            }
            if(cycleCase == 1){
                telemetry.addData("Cycle Case", "^^^SPECIMEN^^^");
            }

            if(triangleCounter == 1){
                telemetry.addData("Current Score Case", scCurrCase + 0.5);
            }
            else if(triangleCounter == 0){
                telemetry.addData("Current Score Case", scCurrCase);

            }
            telemetry.update();


        }
    }

    public void resetSlideEncoders() {
        timer.reset();
        telemetry.addData("Status", "Resetting encoders...");
        telemetry.update();

        // Set target position and mode for the slide motors
        sL.setTargetPosition(0);
        sR.setTargetPosition(0);

        sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motors
        sL.setPower(1);
        sR.setPower(1);

        // Stall detection parameters
        double velocityThreshold = 5; // Adjust based on motor characteristics
        long stallTimeThresholdMillis = 75; // Time threshold for stall detection
        boolean stalled = false;

        // Monitor for stalls
        double startTime = timer.milliseconds();

        while (sL.isBusy() || sR.isBusy()) {
            // Check velocity for stall detection
            double sLVelocity = Math.abs(sL.getVelocity());
            double sRVelocity = Math.abs(sR.getVelocity());

            if (!stalled && (sLVelocity < velocityThreshold || sRVelocity < velocityThreshold)) {
                if (timer.milliseconds() - startTime > stallTimeThresholdMillis) {
                    stalled = true;

                    // Stop the motors and reset encoders
                    sL.setPower(0);
                    sL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sR.setPower(0);
                    sR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    telemetry.addData("Slide 1 Stalled", sL.getCurrentPosition());
                    telemetry.addData("Slide 2 Stalled", sR.getCurrentPosition());
                    break;
                }
            } else {
                // Reset the stall timer if velocity is above the threshold
                startTime = timer.milliseconds();
            }

            telemetry.addData("Slide 1 Velocity", sLVelocity);
            telemetry.addData("Slide 2 Velocity", sRVelocity);
            telemetry.update();
        }

        // Stop motors after reset or completion
        sL.setPower(0);
        sR.setPower(0);

        // Set motors back to encoder mode for regular operation
        sL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Reset complete");
        telemetry.update();
    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower =  x -y -yaw;
        double rightFrontPower =  x +y +yaw;
        double leftBackPower =  x +y -yaw;
        double rightBackPower =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

    }
}