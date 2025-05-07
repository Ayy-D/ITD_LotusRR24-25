package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name="SC SPECIMEN")

public class sc_specimen extends LinearOpMode{
    String placeholder = "----";


    //Directionally named motors based on view from opposite of back odo pod side

    //Control Hub Motors
    private DcMotorEx rightFront = null;
    private DcMotorEx leftFront = null;
    private DcMotorEx rightBack = null;
    private DcMotorEx leftBack= null;

    //Control Hub Servos
    private CRServo inR = null;
    private CRServo inL = null;
    private Servo inTwi = null;
    private Servo inUD = null; // SPM
    private Servo inArmR = null; // SPM
    private Servo inArmL = null; // SPM


    //Expansion Hub Motors

    private DcMotorEx sL = null;
    private DcMotorEx sR = null;

    //Expansion Hub Servos
    private Servo scR = null; //SPM
    private Servo scL = null; //SPM
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
        inR = hardwareMap.get(CRServo.class, "inRight");
        inL = hardwareMap.get(CRServo.class, "inLeft");
        inTwi = hardwareMap.get(Servo.class, "inTwist");
        inTwi.setDirection(Servo.Direction.REVERSE);
        inUD = hardwareMap.get(Servo.class, "inUD");
        inArmR = hardwareMap.get(Servo.class, "inArmR");
        inArmL = hardwareMap.get(Servo.class, "inArmL");

        //Sensors + Cameras

        //Expansion Hub Servos
        scR = hardwareMap.get(Servo.class, "scArmR"); //0.95 goes toward intake, 0 goes outward from robot
        scL = hardwareMap.get(Servo.class, "scArmL"); //same as above
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
        boolean scLastButtonState = false;

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

        scC.setPosition(1);
        resetSlideEncoders();


        while (opModeIsActive())
        {

            drive  = gamepad1.right_stick_y  / (1.25 + gamepad1.right_trigger * 3);  // Reduce drive rate to 27-80%.
            strafe = gamepad1.right_stick_x  / (1.25  + gamepad1.right_trigger * 3);  // Reduce strafe rate to 27-80%.
            turn   = -gamepad1.left_stick_x / (1.5 + gamepad1.right_trigger * 3);  // turn rate 22-67%.
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
                scC.setPosition(1);
                specCount = 0;
            }

            if (((sL.getCurrent(CurrentUnit.AMPS) > 4 || sR.getCurrent(CurrentUnit.AMPS ) > 4) || gamepad2.dpad_down) && scCurrCase == 0) {
                resetSlideEncoders(); // Call the reset function
            }

            //Sample Automation
            if(cycleCase == 0){
                if(rightBumperState && !inLastButtonStateR){  inCurrCase = (inCurrCase + 1) % 10;  }
                inLastButtonStateR = rightBumperState;

                if(leftBumperState && !inLastButtonStateL){   inCurrCase = (inCurrCase - 1 + 10) % 10;   }
                inLastButtonStateL = leftBumperState;

                switch(inCurrCase){

                    case 0: // full collapsed
                        inArmR.setPosition(0.14);
                        inArmL.setPosition(0.14);
                        inUD.setPosition(0.425);
                        inTwi.setPosition(0.25);
                        inR.setPower(0);
                        inL.setPower(0);

                        break;

                    case 1: // intaking halfway
                        sL.setPower(0.8);
                        sL.setTargetPosition(0);
                        sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sR.setPower(0.8);
                        sR.setTargetPosition(0);
                        sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        inArmR.setPosition(0.25);
                        inArmL.setPosition(0.25);
                        inUD.setPosition(0.55);
                        inTwi.setPosition(0.35);
                        inR.setPower(0);
                        inL.setPower(0);

                        scR.setPosition(1);
                        scL.setPosition(1);
                        scUD.setPosition(0.67);

                        break;


                    case 2: // full out - intaking

                        inArmR.setPosition(0.34);
                        inArmL.setPosition(0.34);
                        inUD.setPosition(0.845);
                        inTwi.setPosition(0.58);
                        inR.setPower(-0.75);
                        inL.setPower(0.75);
                        scUD.setPosition(0.67);

                        if(gamepad1.triangle){
                            inR.setPower(0.75);
                            inL.setPower(-0.75);
                        }

                        scR.setPosition(1);
                        scL.setPosition(1);

                        break;

                    case 3: // scoring halfway
                        inArmR.setPosition(0.25);
                        inArmL.setPosition(0.25);
                        inUD.setPosition(0.35);
                        inTwi.setPosition(0.25);
                        inR.setPower(0);
                        inL.setPower(0);

                        scR.setPosition(1);
                        scL.setPosition(1);
                        scUD.setPosition(0.67);

                        break;

                    case 4:// transfer collapse
                        timer.reset();
                        inArmR.setPosition(0.145);
                        inArmL.setPosition(0.145);
                        inUD.setPosition(0.2);
                        inTwi.setPosition(0.25);
                        inR.setPower(0);
                        inL.setPower(0);

                        scR.setPosition(1);
                        scL.setPosition(1);
                        scUD.setPosition(0.67);

                        inCurrCase = 5;
                        break;

                    case 5:
                        if(timer.milliseconds() > 500){
                            inTwi.setPosition(0.58);
                            inUD.setPosition(0.16);
                            inCurrCase = 6;
                        }

                        break;

                    case 6: // transfer
                        timer.reset();

                        inArmR.setPosition(0.145);
                        inArmL.setPosition(0.145);

                        scR.setPosition(1);
                        scL.setPosition(1);
                        scUD.setPosition(0.67);

                        inCurrCase = 7;
                        break;

                    case 7:
                        if(timer.milliseconds() > 250){
                            inR.setPower(0.75);
                            inL.setPower(-0.75);
                        }
                        break;

                    case 8:
                        timer.reset();
                        inTwi.setPosition(0.25);
                        inCurrCase = 9;
                        break;

                    case 9:
                        if(timer.milliseconds() > 500){
                            inCurrCase = 0;
                            scCurrCase = 1;
                        }
                        break;
                }

                if(inCurrCase == 0){
                    if((gamepad2.triangle || gamepad1.triangle) && scCurrCase > 1){
                        scUD.setPosition(0.7);
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

                            sL.setPower(0.9);
                            sL.setTargetPosition(0);
                            sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            sR.setPower(0.9);
                            sR.setTargetPosition(0);
                            sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            scR.setPosition(1);
                            scL.setPosition(1);
                            scUD.setPosition(0.67);
                            triangleCounter = 0;
                            break;

                        case 1:
                            timer.reset();
                            scR.setPosition(0.48);
                            scL.setPosition(0.48);
                            scCurrCase = 2;
                            break;

                        case 2:
                            if(timer.milliseconds() > 250) {
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
                            scR.setPosition(0.48);
                            scL.setPosition(0.48);

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
                        inArmR.setPosition(0.14);
                        inArmL.setPosition(0.14);
                        inUD.setPosition(0.425);
                        inTwi.setPosition(0.25);
                        inR.setPower(0);
                        inL.setPower(0);
                        break;

                    case 1: // intaking halfway
                        inArmR.setPosition(0.25);
                        inArmL.setPosition(0.23);
                        inUD.setPosition(0.65);
                        inTwi.setPosition(0.35);
                        inR.setPower(0);
                        inL.setPower(0);
                        break;


                    case 2: // full out - intaking
                        inArmR.setPosition(0.34);
                        inArmL.setPosition(0.34);
                        inUD.setPosition(0.84);
                        inTwi.setPosition(0.58);
                        inR.setPower(-1);
                        inL.setPower(1);

                        if(gamepad1.triangle){
                            inR.setPower(1);
                            inL.setPower(-1);
                        }
                        break;

                    case 3: // scoring halfway
                        inArmR.setPosition(0.25);
                        inArmL.setPosition(0.23);
                        inUD.setPosition(0.65);
                        inTwi.setPosition(0.25);
                        inR.setPower(0);
                        inL.setPower(0);

                        break;

                    case 4: // human player collapse
                        inArmR.setPosition(0.14);
                        inArmL.setPosition(0.14);
                        inUD.setPosition(0.65);
                        inTwi.setPosition(0.25);
                        inR.setPower(0);
                        inL.setPower(0);
                        break;

                    case 5: // out to hp
                        inArmR.setPosition(0.14);
                        inArmL.setPosition(0.14);
                        inUD.setPosition(0.65);
                        inTwi.setPosition(0.25);
                        inR.setPower(0.7);
                        inL.setPower(-0.7);
                        break;

                }

                if(inCurrCase == 0){
                    if(timer.milliseconds() > 1000 && specCount == 0){
                        scC.setPosition(0.9);
                        specCount = 1;
                    }

                    if(xButtonState && !scLastButtonState && triangleCounter != 1){
                        scCurrCase = (scCurrCase + 1) % 5;
                    }
                    if(xButtonState && !scLastButtonState && triangleCounter == 1){
                        scCurrCase = 0;
                    }

                    scLastButtonState = xButtonState;

                    switch(scCurrCase){
                        case 0: // grab from wall

                            sL.setPower(0.8);
                            sL.setTargetPosition(0);
                            sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            sR.setPower(0.8);
                            sR.setTargetPosition(0);
                            sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            scR.setPosition(0.13);
                            scL.setPosition(0.13);
                            scUD.setPosition(0.92);

                            triangleCounter = 0;

                            break;

                        case 1: // out to score
                            timer.reset();
                            scC.setPosition(0.25);
                            scCurrCase = 2;
                            break;

                        case 2: // wait for delay
                            if(timer.milliseconds() > 250) {
                                scR.setPosition(0.3);
                                scL.setPosition(0.3);
                                scUD.setPosition(0.97);
                                sL.setPower(0.75);
                                sL.setTargetPosition(570);
                                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                sR.setPower(0.75);
                                sR.setTargetPosition(570);
                                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                scCurrCase = 3;
                                break;
                            }
                            break;

                        case 3:
                            if(gamepad2.triangle){ //pull to clip
                                timer.reset();

                                sL.setPower(1);
                                sL.setTargetPosition(20);
                                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                sR.setPower(1);
                                sR.setTargetPosition(20);
                                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                                scR.setPosition(0.16);
                                scL.setPosition(0.16);

                                triangleCounter = 1;
                                scCurrCase = 4;
                            }
                            break;

                        case 4: //claw open
                            if(timer.milliseconds() > 200) {
                                scC.setPosition(1);
                                scCurrCase = 0;
                            }

                            break;

                    }

                }
            }

            //Testing
            if(cycleCase == 2){
                if(gamepad1.cross) {inTwi.setPosition(1); }
                if(gamepad1.triangle) {inTwi.setPosition(0); }
            }

            telemetry.addData("-----", placeholder);

            telemetry.addData("sR Position", sR.getCurrentPosition());
            telemetry.addData("sR Current Draw", sR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("sL Position", sL.getCurrentPosition());
            telemetry.addData("sL Current Draw", sL.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("-----", placeholder);

            if(cycleCase == 0){
                telemetry.addData("Cycle Case", "***SAMPLE***");
            }
            if(cycleCase == 1){
                telemetry.addData("Cycle Case", "^^^SPECIMEN^^^");
            }

            telemetry.addData("Current Score Case", scCurrCase);
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