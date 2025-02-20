package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name="Sri Tele")

public class soloTeleOp extends LinearOpMode{
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
    private DcMotorEx rotR = null;
    private DcMotorEx rotL = null;
    private DcMotorEx sL = null;
    private DcMotorEx sR = null;

    //Expansion Hub Servos
    private Servo scR = null; //SPM
    private Servo scL = null; //SPM
    private Servo scUD = null; //SPM
    private Servo scC = null;

    //Sensors & Cameras
    private ColorSensor color = null;
    int red;
    int blue;
    int green;

    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        //drivetrain power vars
        double  drive = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn = 0;        // Desired turning power/speed (-1 to +1)

        //drivetrain motors
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);


        //Linear Slide Rotation Motors
        rotR = hardwareMap.get(DcMotorEx.class, "rotateR");
        //rotR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rotR.setDirection(DcMotorEx.Direction.REVERSE);


        rotL = hardwareMap.get(DcMotorEx.class, "rotateL");

        //rotL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //rotL.setDirection(DcMotorEx.Direction.REVERSE);



        //LS Motors
        sL = hardwareMap.get(DcMotorEx.class, "slideL");
        sL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sR = hardwareMap.get(DcMotorEx.class, "slideR");
        sR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sR.setDirection(DcMotorEx.Direction.REVERSE);


        //Control Hub Servos (Intake)
        inR = hardwareMap.get(CRServo.class, "inRight");
        inL = hardwareMap.get(CRServo.class, "inLeft");
        inTwi = hardwareMap.get(Servo.class, "inTwist");
        inUD = hardwareMap.get(Servo.class, "inUD");
        inArmR = hardwareMap.get(Servo.class, "inArmR");
        inArmL = hardwareMap.get(Servo.class, "inArmL");

        //Sensors + Cameras
        color = hardwareMap.get(ColorSensor.class, "color");
        red = color.red();
        blue = color.blue();
        green = color.green();

        //Expansion Hub Servos
        scR = hardwareMap.get(Servo.class, "scArmR"); //0.95 goes toward intake, 0 goes outward from robot
        scL = hardwareMap.get(Servo.class, "scArmL"); //same as above
        scUD = hardwareMap.get(Servo.class, "scUD"); //1 is the position for depositing an element, 0.8 for intake, <0.8 to keep it up
        scUD.setDirection(Servo.Direction.REVERSE);
        scC = hardwareMap.get(Servo.class, "scClaw"); //0.27 close, 0.8 open

        //Sample-Specimen Cycle
        int cycleCase = 1; // 0 - Sample, 1 - Specimen

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

        scC.setPosition(0.3);
        resetSlideEncoders();


        while (opModeIsActive())
        {

            drive  = gamepad1.right_stick_y  / (1.3 + gamepad1.right_trigger * 3);  // Reduce drive rate to 44-80%.
            strafe = gamepad1.right_stick_x  / (1.3  + gamepad1.right_trigger * 3);  // Reduce strafe rate to 33-100%.
            turn   = -gamepad1.left_stick_x / (1.5 + gamepad1.right_trigger * 3);  // turn rate 25-50%.
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
                scC.setPosition(0.3);
                specCount = 0;
            }

            if (((sL.getCurrent(CurrentUnit.AMPS) > 4 || sR.getCurrent(CurrentUnit.AMPS ) > 4) || gamepad2.dpad_down) && scCurrCase == 0) {
                resetSlideEncoders(); // Call the reset function
            }


            if(rightBumperState && !inLastButtonStateR){  inCurrCase = (inCurrCase + 1) % 6;  }
            inLastButtonStateR = rightBumperState;

            if(leftBumperState && !inLastButtonStateL){   inCurrCase = (inCurrCase - 1 + 6) % 6;   }
            inLastButtonStateL = leftBumperState;

            //Sample Automation
            if(cycleCase == 0){
                switch(inCurrCase){

                    case 0: // full collapsed
                        inArmR.setPosition(0.14);
                        inArmL.setPosition(0.14);
                        inUD.setPosition(0.4);
                        inTwi.setPosition(0.3);
                        inR.setPower(0);
                        inL.setPower(0);

                        break;

                    case 1: // intaking halfway
                        inArmR.setPosition(0.25);
                        inArmL.setPosition(0.25);
                        inUD.setPosition(0.65);
                        inTwi.setPosition(0.35);
                        inR.setPower(0);
                        inL.setPower(0);

                        scR.setPosition(0.96);
                        scL.setPosition(0.96);
                        scUD.setPosition(0.75);

                        break;


                    case 2: // full out - intaking
                        inArmR.setPosition(0.33);
                        inArmL.setPosition(0.3);
                        inUD.setPosition(0.86);
                        inTwi.setPosition(0.56);
                        inR.setPower(-0.75);
                        inL.setPower(0.75);
                        scUD.setPosition(0.65);

                        if(gamepad1.triangle){
                            inR.setPower(0.75);
                            inL.setPower(-0.75);
                        }

                        scR.setPosition(0.96);
                        scL.setPosition(0.96);

                        break;

                    case 3: // scoring halfway
                        inArmR.setPosition(0.25);
                        inArmL.setPosition(0.25);
                        inUD.setPosition(0.35);
                        inTwi.setPosition(0.35);
                        inR.setPower(0);
                        inL.setPower(0);

                        scR.setPosition(0.98);
                        scL.setPosition(0.98);
                        scUD.setPosition(0.65);

                        break;

                    case 4:// transfer collapse
                        inArmR.setPosition(0.11);
                        inArmL.setPosition(0.135);
                        inUD.setPosition(0.2);
                        inTwi.setPosition(0.35);
                        inR.setPower(0);
                        inL.setPower(0);

                        scR.setPosition(0.98);
                        scL.setPosition(0.98);
                        scUD.setPosition(0.65);

                        break;

                    case 5: // transfer
                        inArmR.setPosition(0.11);
                        inArmL.setPosition(0.135);
                        inUD.setPosition(0.2);
                        inTwi.setPosition(0.37);
                        inR.setPower(0.75);
                        inL.setPower(-0.75);

                        scR.setPosition(0.98);
                        scL.setPosition(0.98);
                        scUD.setPosition(0.65);

                        break;

                }

                if(inCurrCase == 0){
                    if(gamepad2.triangle && scCurrCase > 1){
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
                            rotL.setPower(0.8);
                            rotL.setTargetPosition(100);
                            rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            rotR.setPower(0.8);
                            rotR.setTargetPosition(100);
                            rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            sL.setPower(0.9);
                            sL.setTargetPosition(0);
                            sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            sR.setPower(0.9);
                            sR.setTargetPosition(0);
                            sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            scR.setPosition(0.96);
                            scL.setPosition(0.96);
                            scUD.setPosition(0.75);
                            triangleCounter = 0;
                            break;

                        case 1:
                            inUD.setPosition(0.5);
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

                            rotL.setPower(0.8);
                            rotL.setTargetPosition(100);
                            rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            rotR.setPower(0.8);
                            rotR.setTargetPosition(100);
                            rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

                            rotL.setPower(0.8);
                            rotL.setTargetPosition(100);
                            rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            rotR.setPower(0.8);
                            rotR.setTargetPosition(100);
                            rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                switch(inCurrCase){
                    case 0: // full collapsed - grab from wall
                        inArmR.setPosition(0.14);
                        inArmL.setPosition(0.12);
                        inUD.setPosition(0.3);
                        inTwi.setPosition(0.3);
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
                        inArmR.setPosition(0.33);
                        inArmL.setPosition(0.3);
                        inUD.setPosition(0.86);
                        inTwi.setPosition(0.56);
                        inR.setPower(-1);
                        inL.setPower(1);

                        if(gamepad1.triangle){
                            inR.setPower(1);
                            inL.setPower(-1);
                        }
                        //         //runIntake(red, blue, green);
                        break;

                    case 3: // scoring halfway
                        inArmR.setPosition(0.25);
                        inArmL.setPosition(0.23);
                        inUD.setPosition(0.65);
                        inTwi.setPosition(0.35);
                        inR.setPower(0);
                        inL.setPower(0);

                        break;

                    case 4: // human player collapse
                        inArmR.setPosition(0.14);
                        inArmL.setPosition(0.14);
                        inUD.setPosition(0.65);
                        inTwi.setPosition(0.35);
                        inR.setPower(0);
                        inL.setPower(0);
                        break;

                    case 5: // out to hp
                        inArmR.setPosition(0.14);
                        inArmL.setPosition(0.14);
                        inUD.setPosition(0.65);
                        inTwi.setPosition(0.35);
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
                            rotL.setPower(0.8);
                            rotL.setTargetPosition(100);
                            rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            rotR.setPower(0.8);
                            rotR.setTargetPosition(100);
                            rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                            scC.setPosition(0.3);
                            scCurrCase = 2;
                            break;

                        case 2: // wait for delay
                            if(timer.milliseconds() > 250) {
                                scR.setPosition(0.3);
                                scL.setPosition(0.3);
                                scUD.setPosition(0.97);
                                sL.setPower(0.5);
                                sL.setTargetPosition(570);
                                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                sR.setPower(0.5);
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
                            if(timer.milliseconds() > 250) {
                                scC.setPosition(0.9);
                                scCurrCase = 0;
                            }

                            break;

                    }

                }
            }


            telemetry.addData("rotR Position", rotR.getCurrentPosition());
            //telemetry.addData("rotR Current Draw", rotR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rotL Position", rotL.getCurrentPosition());
            //telemetry.addData("rotL Current Draw", rotL.getCurrent(CurrentUnit.AMPS));


            telemetry.addData("-----", placeholder);

            telemetry.addData("sR Position", sR.getCurrentPosition());
            //telemetry.addData("sR Current Draw", sR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("sL Position", sL.getCurrentPosition());
            //telemetry.addData("sL Current Draw", sL.getCurrent(CurrentUnit.AMPS));

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


    public void runIntake(int red, int blue, int green){
        telemetry.addData("Intake ON!", placeholder);
        telemetry.update();
        //Intake wheels
        // if(gamepad1.triangle || gamepad1.y){ //intaking
        //     inR.setPower(-1);
        //     inL.setPower(1);
        // }
        // if(gamepad1.circle || gamepad1.b){ //depositing
        //     inR.setPower(0.1);
        //     inL.setPower(-0.1);
        // }
        // if(gamepad1.cross ||gamepad1.a){ //rest

        // }
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