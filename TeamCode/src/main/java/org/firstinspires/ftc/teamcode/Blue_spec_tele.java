package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name="spec - blue")

public class Blue_spec_tele extends LinearOpMode{

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
    private Servo inPiv = null;
    private Servo scC = null;

    //Sensors & Cameras
    private ColorSensor col = null;

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
        inWR = hardwareMap.get(CRServo.class, "inWRight");
        inWL = hardwareMap.get(CRServo.class, "inWLeft");

        //Sensors + Cameras
        col = hardwareMap.get(ColorSensor.class, "color");

        //Expansion Hub Servos
        scArm = hardwareMap.get(Servo.class, "scArmL"); //same as above
        scUD = hardwareMap.get(Servo.class, "scUD"); //1 is the position for depositing an element, 0.8 for intake, <0.8 to keep it up
        scUD.setDirection(Servo.Direction.REVERSE);
        inPiv = hardwareMap.get(Servo.class, "inPivot"); //1 full up, 0.55 full down
        inPiv.setDirection(Servo.Direction.REVERSE);
        scC = hardwareMap.get(Servo.class, "scClaw"); //0.27 close, 0.8 open

        //Sample-Specimen Cycle
        int cycleCase = 1; // 0 - Sample, 1 - Specimen, 2 - Individual System Testing

        //Intake/Scoring Trigger Cycle Variables
        int inCurrCase = 0;
        boolean inLastButtonStateR = false;
        boolean inLastButtonStateL = false;
        boolean red = false;; //red alliance - true, blue alliance - false


        // Linear Slide Cycle Variables
        int scCurrCase = 0;
        boolean scLastButtonState = false;

        //Linear Slide Reset Counter
        boolean hasResetEncoders = false;

        //Scoring Arm Counter - SAMPLE AUTOMATION ONLY
        int triangleCounter = 0;

        //Specimen Variables
        int specCount = 0;
        int specScoring = 0;

        waitForStart();
        telemetry.addData(">>", "Wait for Start");
        telemetry.update();
        if (isStopRequested()) return;

        scC.setPosition(0.3);
        scArm.setPosition(1);
        scUD.setPosition(0.67);

        inR.setPosition(0.32);
        inB.setPosition(0.32);
        inUD.setPosition(0.67);
        inPiv.setPosition(0.6);
        inWR.setPower(0);
        inWL.setPower(-0);


        while (opModeIsActive())
        {

            drive  = gamepad1.left_stick_y  / (1.4 + gamepad1.right_trigger * 0.5);  // Reduce drive rate to 27-80%.
            strafe = gamepad1.left_stick_x  / (1.4  + gamepad1.right_trigger * 0.5);  // Reduce strafe rate to 27-80%.
            turn   = -gamepad1.right_stick_x / (2 + gamepad1.right_trigger * 0.5);  // turn rate 22-67%.
            moveRobot(drive, strafe, turn);


            boolean xButtonState = gamepad2.cross || gamepad1.cross || gamepad1.a ||gamepad2.a; // linear slides++
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
            if (((sL.getCurrent(CurrentUnit.AMPS) > 3.5 || sR.getCurrent(CurrentUnit.AMPS ) > 3.5) || gamepad2.dpad_down) && (sL.getCurrentPosition() < 50 && sR.getCurrentPosition() < 50) && scCurrCase == 0) {
                resetSlideEncoders(); // Call the reset function
            }

            //Sample Automation
            if(cycleCase == 0){
                int transferFrom = 0;
                int numCycles = 7;
                if(rightBumperState && !inLastButtonStateR){
                    inCurrCase = (inCurrCase + 1) % (numCycles + 1);
                    scCurrCase = 0;

                }
                inLastButtonStateR = rightBumperState;

                if(leftBumperState && !inLastButtonStateL){
                    inCurrCase = (inCurrCase - 1 + (numCycles + 1)) % (numCycles + 1);
                    scCurrCase = 0;

                }
                inLastButtonStateL = leftBumperState;


                switch(inCurrCase){
                    case 0: // full collapsed
                        inR.setPosition(0.32);
                        inB.setPosition(0.32);
                        if(scCurrCase == 0 && (sL.getCurrentPosition() < 100 || sR.getCurrentPosition() < 100)){
                            inUD.setPosition(0.67);
                            inPiv.setPosition(0.6);
                        }
                        inWR.setPower(0);
                        inWL.setPower(-0);
                        break;

                    case 1: //prep intake pos
                        transferFrom = 1;
                        inR.setPosition(0.39);
                        inB.setPosition(0.39);

                        if(gamepad1.left_trigger > 0.5){
                            inUD.setPosition(0.45);
                            inPiv.setPosition(0.225);

                        }
                        else{
                            inPiv.setPosition(0.16);
                            inUD.setPosition(0.5);
                        }
                        if(gamepad1.triangle || col.red() > 500){
                            inPiv.setPosition(0.5);
                            inWR.setPower(1);
                            inWL.setPower(-1);
                        }
                        else{
                            inWR.setPower(-0.6);
                            inWL.setPower(0.6);

                            if(col.blue() > 500 || col.green() > 500){
                                timer.reset();
                                inCurrCase = 3;
                            }
                        }

                        break;

                    case 2: // intaking until distance find element - push outminor/major switch
                        timer.reset();
                        transferFrom = 2;

                        inB.setPosition(0.52);
                        inR.setPosition(0.52);

                        if (gamepad1.left_trigger > 0.5) {
                            inUD.setPosition(0.42);
                            inPiv.setPosition(0.18);
                        } else {
                            inPiv.setPosition(0.125);
                            inUD.setPosition(0.5);
                        }

                        if (gamepad1.triangle || col.red() > 500) {
                            inPiv.setPosition(0.5);
                            inWR.setPower(1);
                            inWL.setPower(-1);
                        } else {
                            inWR.setPower(-0.6);
                            inWL.setPower(0.6);
                        }

                        if(col.blue() > 500 || col.green() > 500){

                            timer.reset();
                            inCurrCase = 3;
                        }

                        break;


                    case 3: //prep transfer
                        if(transferFrom == 2){
                            inR.setPosition(0.52);
                            inB.setPosition(0.52);
                        }
                        if(transferFrom == 1){
                            inR.setPosition(0.4);
                            inB.setPosition(0.4);
                        }

                        inUD.setPosition(0.65);
                        inPiv.setPosition(0.4);
                        inWR.setPower(0);
                        inWL.setPower(0);

                        if(timer.milliseconds() > 400){
                            timer.reset();
                            inCurrCase = 4;
                        }
                        break;

                    case 4: // transfer collapse
                        inR.setPosition(0.32);
                        inB.setPosition(0.32);
                        inUD.setPosition(0.65);
                        inPiv.setPosition(0.85);
                        if(timer.milliseconds() > 400){
                            inCurrCase = 5;
                        }
                        break;

                    case 5: // outtake to bucket until distance senses nothing in intake
                        inWR.setPower(1);
                        inWL.setPower(-1);

                        if(col.blue() < 550 && col.green() < 550){
                            inCurrCase = 6;
                        }
                        break;

                    case 6:
                        timer.reset();
                        inR.setPosition(0.37);
                        inB.setPosition(0.37);

                        inUD.setPosition(0.8);
                        inPiv.setPosition(0.6);
                        inCurrCase = 7;
                        break;

                    case 7:

                        if(timer.milliseconds() > 50){
                            scCurrCase = 1;
                            inCurrCase = 0;
                        }
                        break;

                }

                if(inCurrCase == 0){
                    int lower = 0;

                    if((gamepad2.triangle || gamepad1.triangle) && scCurrCase > 1){
                        scUD.setPosition(0.775);
                        triangleCounter = 1;
                    }
                    if(xButtonState && !scLastButtonState && triangleCounter != 1){
                        scCurrCase = (scCurrCase + 1) % 5;

                    }
                    if(xButtonState && !scLastButtonState && triangleCounter == 1){
                        scCurrCase = 0;
                    }

                    scLastButtonState = xButtonState;


                    switch(scCurrCase){
                        case 0:
                            timer.reset();
                            lower = 0;

                            sL.setPower(0.7);
                            sL.setTargetPosition(0);
                            sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            sR.setPower(0.7);
                            sR.setTargetPosition(0);
                            sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            scArm.setPosition(1);
                            scUD.setPosition(0.76);
                            triangleCounter = 0;
                            break;

                        case 1:
                            scArm.setPosition(0.48);
                            inUD.setPosition(0.5);
                            scCurrCase = 2;
                            break;

                        case 2:
                            if(timer.milliseconds() > 250) {
                                scUD.setPosition(0.4);
                                scCurrCase = 3;
                            }

                            break;

                        case 3:
                            if(gamepad1.dpad_down){
                                lower += 1;
                            }

                            if(lower == 0){
                                sL.setPower(1);
                                sL.setTargetPosition(1900);
                                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                sR.setPower(1);
                                sR.setTargetPosition(1900);
                                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            }
                            else{
                                sL.setPower(1);
                                sL.setTargetPosition(750);
                                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                sR.setPower(1);
                                sR.setTargetPosition(750);
                                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            }
                            break;
                    }

                }
            }

            //Specimen Automation
            if(cycleCase == 1){
                if(gamepad1.cross && specScoring == 0){
                    specScoring = 1;
                }
                if(gamepad1.cross && specScoring == 1){
                    specScoring = 0;
                }

                if(specScoring == 0) {
                    scArm.setPosition(0.48);
                    inUD.setPosition(0.4);
                    int numCycles = 5; // # of last case
                    if (rightBumperState && !inLastButtonStateR) {
                        inCurrCase = (inCurrCase + 1) % (numCycles + 1);
                        scCurrCase = 0;

                    }
                    inLastButtonStateR = rightBumperState;

                    if (leftBumperState && !inLastButtonStateL) {
                        inCurrCase = (inCurrCase - 1 + (numCycles + 1)) % (numCycles + 1);
                        scCurrCase = 0;

                    }
                    inLastButtonStateL = leftBumperState;


                    switch (inCurrCase) {

                        case 0: // full collapsed
                            inR.setPosition(0.32);
                            inB.setPosition(0.32);

                            inUD.setPosition(0.5);
                            inPiv.setPosition(0.5);

                            inWR.setPower(0);
                            inWL.setPower(0);
                            break;

                        case 1: //prep intake pos
                            inR.setPosition(0.39);
                            inB.setPosition(0.39);

                            if (gamepad1.left_trigger > 0.5) {
                                inUD.setPosition(0.42);
                                inPiv.setPosition(0.18);

                            } else {
                                inPiv.setPosition(0.125);
                                inUD.setPosition(0.5);
                            }
                            if( gamepad1.triangle || ( col.red() > 500 || ( col.green() > col.red() && col.green() > 500 ) ) ){
                                inPiv.setPosition(0.5);
                                inWR.setPower(1);
                                inWL.setPower(-1);
                            }
                            else{
                                inWR.setPower(-0.6);
                                inWL.setPower(0.6);

                                if(col.blue() > col.green() && col.blue() > 500){
                                    timer.reset();
                                    inCurrCase = 3;
                                }
                            }

                            break;

                        case 2: // intaking until distance find element - push outminor/major switch
                            timer.reset();

                            inB.setPosition(0.46);
                            inR.setPosition(0.46);

                            if (gamepad1.left_trigger > 0.5) {
                                inUD.setPosition(0.42);
                                inPiv.setPosition(0.18);
                            } else {
                                inPiv.setPosition(0.125);
                                inUD.setPosition(0.5);
                            }

                            if( gamepad1.triangle || ( col.red() > 500 || ( col.green() > col.red() && col.green() > 500 ) ) ){
                                inPiv.setPosition(0.5);
                                inWR.setPower(1);
                                inWL.setPower(-1);
                            }
                            else{
                                inWR.setPower(-0.6);
                                inWL.setPower(0.6);

                                if(col.blue() > col.green() && col.blue() > 500){
                                    timer.reset();
                                    inCurrCase = 3;
                                }
                            }

                            break;


                        case 3: // intake collapse
                            inR.setPosition(0.34);
                            inB.setPosition(0.34);

                            inUD.setPosition(0.5);
                            inPiv.setPosition(0.5);

                            inWR.setPower(0);
                            inWL.setPower(0);
                            break;

                        case 4: // HP push
                            timer.reset();
                            inR.setPosition(0.46);
                            inB.setPosition(0.46);

                            inUD.setPosition(0.5);
                            inPiv.setPosition(0.5);

                            inCurrCase = 5;
                            break;

                        case 5: // outake to HP

                            if (timer.milliseconds() > 200) {
                                inWR.setPower(1);
                                inWL.setPower(-1);
                            }
                            if(col.blue() < 300 && timer.milliseconds() > 500){
                                inCurrCase = 0;
                            }
                            break;

                    }


                }

                if(specScoring == 1){
                    int numCycles = 5; // # of last case
                    if (rightBumperState && !inLastButtonStateR) {
                        inCurrCase = (inCurrCase + 1) % (numCycles + 1);
                        scCurrCase = 0;

                    }
                    inLastButtonStateR = rightBumperState;

                    if (leftBumperState && !inLastButtonStateL) {
                        inCurrCase = (inCurrCase - 1 + (numCycles + 1)) % (numCycles + 1);
                        scCurrCase = 0;

                    }
                    inLastButtonStateL = leftBumperState;

                    switch (inCurrCase) {
                        case 0: // full collapsed
                            inR.setPosition(0.32);
                            inB.setPosition(0.32);

                            inUD.setPosition(0.5);
                            inPiv.setPosition(0.5);

                            inWR.setPower(0);
                            inWL.setPower(0);

                            sL.setPower(0.7);
                            sL.setTargetPosition(0);
                            sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            sR.setPower(0.7);
                            sR.setTargetPosition(0);
                            sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            scArm.setPosition(0.6);
                            scUD.setPosition(0.76);
                            scC.setPosition(0.3);

                            break;

                        case 1: //full out intake
                            inR.setPosition(0.46);
                            inB.setPosition(0.46);

                            inPiv.setPosition(0.7);
                            inUD.setPosition(0.2);

                            inWR.setPower(-0.4);
                            inWL.setPower(0.4);

                            if (col.blue() > 500) {
                                inCurrCase = 2;
                            }

                            break;

                        case 2: // collapse, prep transfer
                            timer.reset();
                            inB.setPosition(0.32);
                            inR.setPosition(0.32);

                            inUD.setPosition(0.67);
                            inPiv.setPosition(0.6);

                            inWR.setPower(0);
                            inWL.setPower(0);

                            scC.setPosition(0.5);

                            inCurrCase = 3;
                            break;


                        case 3: // delay, then transfer

                            scArm.setPosition(0.6);
                            scUD.setPosition(0.76);

                            if(timer.milliseconds() > 500){
                                scC.setPosition(0.3);
                            }
                            if(timer.milliseconds() > 800){
                                inWR.setPower(0.4);
                                inWL.setPower(-0.4);
                                inB.setPosition(0.39);
                                inR.setPosition(0.39);
                                inCurrCase = 4;
                            }

                            break;

                        case 4: // up to score

                            sL.setPower(0.9);
                            sL.setTargetPosition(100);
                            sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            sR.setPower(0.9);
                            sR.setTargetPosition(100);
                            sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            scUD.setPosition(0.9);


                            if(sL.getCurrentPosition() > 50){
                                inB.setPosition(0.32);
                                inR.setPosition(0.32);

                                inUD.setPosition(0.67);
                                inPiv.setPosition(0.6);

                                inWR.setPower(0);
                                inWL.setPower(0);
                            }

                            if(gamepad1.right_bumper){
                                timer.reset();
                                inCurrCase = 5;
                            }

                            break;

                        case 5: // clip and reset
                            scArm.setPosition(0.3);

                            if (timer.milliseconds() > 600) {
                                scC.setPosition(0.5);
                                scArm.setPosition(0.6);
                                inCurrCase = 0;
                            }

                            break;

                    }
                }
            }

            telemetry.addData("fL Current Draw", leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("fR Current Draw", rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("bL Current Draw", leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("bR Current Draw", rightBack.getCurrent(CurrentUnit.AMPS));

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

            telemetry.addData("-----", placeholder);

            telemetry.addData("Color", ((OpticalDistanceSensor) col).getLightDetected());
            telemetry.addData("Red", col.red());
            telemetry.addData("Green", col.green());
            telemetry.addData("Blue", col.blue());

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