package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@Autonomous(name = "BLUE--SAMPLE", group = "Autonomous")
public class blue_sample extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();

    int sub1YESNO = 0;
    int sub2YESNO = 0;



    //Claw Components
    public class Claw {
        private Servo scC;
        public Claw(HardwareMap hardwareMap) {
            scC = hardwareMap.get(Servo.class, "scClaw"); //0.35 close | 0.9 open
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                scC.setPosition(0.35);
                return false;
            }
        }
        public Action closeClaw() {
            return new Claw.CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                scC.setPosition(0.9);
                return false;
            }
        }
        public Action openClaw() {
            return new Claw.OpenClaw();
        }

    }

    //Rotation Arm Components
    public class rotation {
        private DcMotorEx rotR;
        private DcMotorEx rotL;

        public rotation(HardwareMap hardwareMap) {
            rotR = hardwareMap.get(DcMotorEx.class, "rotateR");
            rotL = hardwareMap.get(DcMotorEx.class, "rotateL");

            rotR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rotR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rotR.setDirection(DcMotorEx.Direction.REVERSE);

            rotL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rotL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }



        public class rotationBase implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotL.setPower(0.4);
                rotR.setPower(0.4);
                rotL.setTargetPosition(0);
                rotR.setTargetPosition(0);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                telemetry.addData("rotR Position", rotR.getCurrentPosition());
                telemetry.addData("rotR Current Draw", rotR.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("rotL Position", rotL.getCurrentPosition());
                telemetry.addData("rotL Current Draw", rotL.getCurrent(CurrentUnit.AMPS));

                return false;
            }
        }public Action rotationBasePos() {
            return new rotation.rotationBase();
        }

        public class rotationUP implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotL.setPower(0.6);
                rotR.setPower(0.6);
                rotL.setTargetPosition(100);
                rotR.setTargetPosition(100);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                telemetry.addData("rotR Position", rotR.getCurrentPosition());
                telemetry.addData("rotR Current Draw", rotR.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("rotL Position", rotL.getCurrentPosition());
                telemetry.addData("rotL Current Draw", rotL.getCurrent(CurrentUnit.AMPS));

                return false;
            }
        }
        public Action rotationUPPos() {
            return new rotation.rotationUP();
        }
    }

    //Linear Slide components
    public class LS_Scoring {
        private Servo scL;
        private Servo scR;
        private Servo scUD;
        private Servo scC;

        private DcMotorEx sL;
        private DcMotorEx sR;


        public LS_Scoring(HardwareMap hardwareMap) {
            scL = hardwareMap.get(Servo.class, "scArmL"); //0.95 goes toward intake, 0 goes outward from robot
            scR = hardwareMap.get(Servo.class, "scArmR"); //0.95 goes toward intake, 0 goes outward from robot
            scUD = hardwareMap.get(Servo.class, "scUD"); //1 is the position for depositing an element, 0.8 for intake, <0.8 to keep it up
            scUD.setDirection(Servo.Direction.REVERSE);
            scC = hardwareMap.get(Servo.class, "scClaw"); //0.35 close, 0.9 open)

            sL = hardwareMap.get(DcMotorEx.class, "slideL");
            sR = hardwareMap.get(DcMotorEx.class, "slideR");

            //RUN Encoders
            sL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //RESET Encoders
            sL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //SLIDE BRAKE Behavior
            sR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //SLIDE Set Direction
            sL.setDirection(DcMotorSimple.Direction.FORWARD);
            sR.setDirection(DcMotorSimple.Direction.REVERSE);


        }

        public class LS_SPECBase implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                scL.setPosition(0.12);
                scR.setPosition(0.12);
                scUD.setPosition(0.9);

                sL.setPower(0.85);
                sR.setPower(0.85);
                sL.setTargetPosition(5);
                sR.setTargetPosition(5);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
        public  Action LS_SPECBasePos() {
            return new LS_Scoring.LS_SPECBase();
        }

        public class LS_SPECScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                scR.setPosition(0.48);
                scL.setPosition(0.48);
                scUD.setPosition(0.4);

                sL.setPower(0.9);
                sL.setTargetPosition(1835);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setPower(0.9);
                sR.setTargetPosition(1820);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }
        public Action LS_SPECScorePos() {
            return new LS_Scoring.LS_SPECScore();
        }

        public class LS_SPECPull implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                scR.setPosition(0.12);
                scL.setPosition(0.12);
                scUD.setPosition(0.85);

                sL.setPower(0.8);
                sR.setPower(0.8);
                sL.setTargetPosition(550);
                sR.setTargetPosition(550);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
        public Action LS_SPECPullPos() {
            return new LS_Scoring.LS_SPECPull();
        }

        public class LS_SAMPLEScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                scR.setPosition(0.48);
                scL.setPosition(0.48);

                sL.setPower(0.85);
                sL.setTargetPosition(1820);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setPower(0.85);
                sR.setTargetPosition(1820);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(time.milliseconds() > 250) {
                    scUD.setPosition(0.4);
                }



                return false;
            }
        }
        public Action LS_SAMPLEScorePos() {
            return new LS_Scoring.LS_SAMPLEScore();
        }

        public class LS_BucketIntakePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                scL.setPosition(0.96);
                scR.setPosition(0.96);
                scUD.setPosition(0.58);
                return false;
            }
        }
        public Action LS_BucketIntakePos() { return new LS_Scoring.LS_BucketIntakePos(); }

        public class LS_BucketTip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                scUD.setPosition(0.7);
                return false;
            }
        }
        public Action LS_BucketTipPos() { return new LS_BucketTip(); }

        public class LS_TeleOp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                sL.setPower(0.9);
                sL.setTargetPosition(5);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setPower(0.9);
                sR.setTargetPosition(5);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                scR.setPosition(0.98);
                scL.setPosition(0.98);
                scUD.setPosition(0.75);
                scC.setPosition(0.35);

                return false;
            }
        }
        public Action LS_TeleOpPos() { return new LS_Scoring.LS_TeleOp(); }
    }

    //Intake components
    public class Intake {
        private CRServo inR;
        private CRServo inL;

        private Servo inUD;
        private Servo inArmR;
        private Servo inArmL;
        private Servo inTwist;

        public Intake(HardwareMap hardwareMap) {
            inR = hardwareMap.get(CRServo.class, "inRight");
            inL = hardwareMap.get(CRServo.class, "inLeft");
            inUD = hardwareMap.get(Servo.class, "inUD");
            inArmL = hardwareMap.get(Servo.class, "inArmL");
            inArmR = hardwareMap.get(Servo.class, "inArmR");
            inTwist = hardwareMap.get(Servo.class, "inTwist");

        }

        public class IntakeInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(0);
                inL.setPower(0);
                inUD.setPosition(0.3);
                inArmL.setPosition(0.14);
                inArmR.setPosition(0.14);
                inTwist.setPosition(0.3);
                return false;
            }
        }
        public Action intakeInitPos() {
            return new Intake.IntakeInit();
        }


        public class IntakeBase implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmL.setPosition(0.14);
                inArmR.setPosition(0.14);
                inUD.setPosition(0.65);
                inTwist.setPosition(0.35);

                return false;
            }
        }
        public Action intakeBasePos() { return new Intake.IntakeBase(); }

        public class IntakeHalfway implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.25);
                inArmL.setPosition(0.25);
                inUD.setPosition(0.5);
                inTwist.setPosition(0.35);

                return false;
            }
        }
        public Action intakeHalfwayPos() { return new Intake.IntakeHalfway(); }

        public class IntakeFullOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.33);
                inArmL.setPosition(0.33);
                inUD.setPosition(0.835);
                inTwist.setPosition(0.56);
                ;
                return false;
            }
        }
        public Action intakeFullOutPos() { return new Intake.IntakeFullOut(); }

        public class IntakeTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inUD.setPosition(0.25);
                new SleepAction(0.2);
                inArmR.setPosition(0.135);
                inArmL.setPosition(0.135);
                inTwist.setPosition(0.35);

                return false;
            }

        }
        public Action intakeTransferPos() { return new Intake.IntakeTransfer(); }



        public class IntakeWheelsIN implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(-0.65);
                inL.setPower(0.65);
                return false;
            }
        }
        public Action intakeWheelsIN() { return new Intake.IntakeWheelsIN(); }

        public class IntakeWheelsOFF implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(0);
                inL.setPower(0);
                return false;
            }
        }
        public Action intakeWheelsOFF() { return new Intake.IntakeWheelsOFF(); }

        public class IntakeWheelsOUT implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(0.6);
                inL.setPower(-0.6);
                return false;
            }
        }
        public Action intakeWheelsOUT() { return new Intake.IntakeWheelsOUT(); }
    }




    @Override
    public void runOpMode() {
        double[] xPose = {
                37.25, // initial pose — 0
                55, // scoring pose 1 — 1

                52, // prep ground pick 1 -- 2
                51, // ground pick 1 — 3

                56, // prep ground pick 2 — 4
                56, // ground pick 2 — 5

                57, // prep ground pick 3 — 6
                57, // ground pick 3 — 7

                40, // prep sub pick 1 - 8
                25, // sub pick 1 - 9

                40, // sub to mid score 4 - 10

                40, // prep sub pick 2 - 11
                25, // sub pick 2 - 12

                40  // sub to mid score 4 - 13


        };

        double[] yPose = {
                61, // initial pose — 0
                55, // scoring pose 1 — 1

                52, // prep ground pick 1 — 2
                48, // ground pick 1 — 3

                52, // prep ground pick 2 — 4
                48, // ground pick 2 — 5

                50, // prep ground pick 3 — 6
                48, // ground pick 3 — 7

                12, // prep sub pick 1 - 8
                12, // sub pick 1 - 9

                12, // sub to mid score 4 - 10

                8,  // prep sub pick 1 - 8
                8,  // sub pick 1 - 9

                8   // sub to mid score 4 - 10
        };

        double[] angles = {
                Math.toRadians(90), // initial pose — 0
                Math.toRadians(45), // scoring pose — 1

                Math.toRadians(82.5), // prep ground pick 1 — 2
                Math.toRadians(82.5), // ground pick 1 — 3

                Math.toRadians(92), // prep ground pick 2 — 4
                Math.toRadians(92), // ground pick 2 — 5

                Math.toRadians(115), // prep ground pick 3 — 6
                Math.toRadians(115), // ground pick 3 — 7

                Math.toRadians(0), // prep sub pick 1 - 8
                Math.toRadians(0), // sub pick 1 - 9

                Math.toRadians(25), // sub to mid score 4 - 10

                Math.toRadians(0), // prep sub pick 2 - 11
                Math.toRadians(0), // sub pick 3 - 12

                Math.toRadians(45)  // sub to mid score 5 - 13
        };

        Pose2d initialPose = new Pose2d(xPose[0], yPose[0], angles[0]);

        Pose2d scoringPose = new Pose2d(xPose[1], yPose[1], angles[1]);

        Pose2d prepgroundpick1 = new Pose2d(xPose[2], yPose[2], angles[2]);

        Pose2d pickup1 = new Pose2d(xPose[3], yPose[3], angles[3]);

        Pose2d prepgroundpick2 = new Pose2d(xPose[4], yPose[4], angles[4]);

        Pose2d pickup2 = new Pose2d(xPose[5], yPose[5], angles[5]);

        Pose2d prepgroundpick3 = new Pose2d(xPose[6], yPose[6], angles[6]);

        Pose2d pickup3 = new Pose2d(xPose[7], yPose[7], angles[7]);

        Pose2d prepsubpick1 = new Pose2d(xPose[8], yPose[8], angles[8]);

        Pose2d subPick1 = new Pose2d(xPose[9], yPose[9], angles[9]);

        Pose2d midScore1 = new Pose2d(xPose[10], yPose[10], angles[10]);

        Pose2d prepsubpick2 = new Pose2d(xPose[11], yPose[11], angles[11]);

        Pose2d subPick2 = new Pose2d(xPose[12], yPose[12], angles[12]);

        Pose2d midScore2 = new Pose2d(xPose[13], yPose[13], angles[13]);


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Claw claw = new Claw(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        LS_Scoring scoring = new LS_Scoring(hardwareMap);
        rotation rotation = new rotation(hardwareMap);



        //init to score preload
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(52, 52), angles[1])
                .strafeToLinearHeading(new Vector2d(xPose[1], yPose[1]), angles[1]);

        //score preload to prep ground pick 1
        TrajectoryActionBuilder tab2 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(xPose[2], yPose[2]), angles[2]);

        //prep ground pick 1 to ground pick 1
        TrajectoryActionBuilder tab3 = drive.actionBuilder(prepgroundpick1)
                .strafeToLinearHeading(new Vector2d(xPose[3], yPose[3]), angles[3]);

        //ground pick 1 to score 1
        TrajectoryActionBuilder tab4 = drive.actionBuilder(pickup1)
                .strafeToLinearHeading(new Vector2d(52, 52), angles[1])
                .strafeToLinearHeading(new Vector2d(xPose[1], yPose[1]), angles[1]);

        //score 1 to prep ground pick 2
        TrajectoryActionBuilder tab5 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(xPose[4], yPose[4]), angles[4]);

        //prep ground pick 2 to ground pick 2
        TrajectoryActionBuilder tab6 = drive.actionBuilder(prepgroundpick2)
                .strafeToLinearHeading(new Vector2d(xPose[5], yPose[5]), angles[5]);

        //ground pick 2 to score 2
        TrajectoryActionBuilder tab7 = drive.actionBuilder(pickup2)
                .strafeToLinearHeading(new Vector2d(52, 52), angles[1])
                .strafeToLinearHeading(new Vector2d(xPose[1], yPose[1]), angles[1]);

        //score 2 to prep ground pick 3
        TrajectoryActionBuilder tab8 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(xPose[6], yPose[6]), angles[6]);

        //prep ground pick 3 to ground pick 3
        TrajectoryActionBuilder tab9 = drive.actionBuilder(prepgroundpick3)
                .strafeToLinearHeading(new Vector2d(xPose[7], yPose[7]), angles[7]);

        //ground pick 3 to score 3
        TrajectoryActionBuilder tab10 = drive.actionBuilder(pickup3)
                .strafeToLinearHeading(new Vector2d(52, 52), angles[1])
                .strafeToLinearHeading(new Vector2d(xPose[1], yPose[1]), angles[1]);

        //score 3 to prep sub pick 1
        TrajectoryActionBuilder tab11 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(xPose[8], yPose[8]), angles[8]);

        //prep sub pick 1 to sub pick 1
        TrajectoryActionBuilder tab12 = drive.actionBuilder(prepsubpick1)
                .strafeToLinearHeading(new Vector2d(xPose[9], yPose[9]), angles[9]);

        //sub pick 1 to mid score 4
        TrajectoryActionBuilder tab13 = drive.actionBuilder(subPick1)
                .strafeToLinearHeading(new Vector2d(xPose[10], yPose[10]), angles[10]);

        //mid score 4 to score 4
        TrajectoryActionBuilder tab14 = drive.actionBuilder(midScore1)
                .strafeToLinearHeading(new Vector2d(52, 52), angles[1])
                .strafeToLinearHeading(new Vector2d(xPose[1], yPose[1]), angles[1]);

        //score 4 to prep sub pick 2
        TrajectoryActionBuilder tab15 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(xPose[11], yPose[11]), angles[11]);

        //prep sub pick 2 to sub pick 2
        TrajectoryActionBuilder tab16 = drive.actionBuilder(prepsubpick2)
                .strafeToLinearHeading(new Vector2d(xPose[12], yPose[12]), angles[12]);

        //sub pick 2 to mid score 4
        TrajectoryActionBuilder tab17 = drive.actionBuilder(subPick2)
                .strafeToLinearHeading(new Vector2d(xPose[13], yPose[13]), angles[13]);

        //mid score 4 to score 4
        TrajectoryActionBuilder tab18 = drive.actionBuilder(midScore2)
                .strafeToLinearHeading(new Vector2d(52, 52), angles[1])
                .strafeToLinearHeading(new Vector2d(xPose[1], yPose[1]), angles[1]);

        // score 4 to park
        Action TrajectoryActionCloseOut = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(55, 16), Math.toRadians(150))
                .strafeToLinearHeading(new Vector2d(32, 12), Math.toRadians(180))
                .build();



        Actions.runBlocking(rotation.rotationBasePos());
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(intake.intakeInitPos());

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("Attempt Sub1? X/A-CONFIRM, Y/∆-cancel", sub1YESNO);
            telemetry.addData("Attempt Sub2? DP_DOWN-CONFIRM, DP_UP-cancel", sub2YESNO);
            telemetry.update();

            if (gamepad1.a || gamepad2.a || gamepad1.cross || gamepad2.cross) { sub1YESNO = 1; }
            if (gamepad1.y || gamepad2.y || gamepad1.triangle || gamepad2.triangle) { sub1YESNO = 0; }
            if (gamepad1.dpad_down || gamepad2.dpad_down) { sub2YESNO = 1; }
            if (gamepad1.dpad_up || gamepad2.dpad_up) { sub2YESNO = 0; }
        }
        // Wait for the start signal
        waitForStart();

        if (isStopRequested()) return;



        Action initToScorePreload = tab1.build();
        Action scorePreloadToPrep1 = tab2.build();
        Action prep1ToPickup1 = tab3.build();
        Action pickup1ToScore1 = tab4.build();
        Action score1ToPrep2 = tab5.build();
        Action prep2ToPickup2 = tab6.build();
        Action pickup2ToScore2 = tab7.build();
        Action score2ToPrep3 = tab8.build();
        Action prep3ToPickup3 = tab9.build();
        Action pickup3ToScore3 = tab10.build();
        Action score3ToPrepSub1 = tab11.build();
        Action prepSub1ToSubPick1 = tab12.build();
        Action subPick1ToMidScore4 = tab13.build();
        Action midScore4ToScore4 = tab14.build();
        Action score4ToPrepSub2 = tab15.build();
        Action prepSub2ToSubPick2 = tab16.build();
        Action subPick2ToMidScore5 = tab17.build();
        Action midScore5ToScore5 = tab18.build();



        // Score Preload
        Actions.runBlocking(
                new SequentialAction(
                        rotation.rotationUPPos(),
                        intake.intakeHalfwayPos(),
                        new SleepAction(1),
                        scoring.LS_SAMPLEScorePos(),
                        initToScorePreload,
                        new SleepAction(0.2),
                        scoring.LS_BucketTipPos()

                )
        );

        // Prep Ground Pick 1 - Ground Pick 1
        Actions.runBlocking(
                new SequentialAction(
                        //Prep
                        scoring.LS_TeleOpPos(),
                        intake.intakeWheelsIN(),
                        intake.intakeHalfwayPos(),
                        new SleepAction(0.01),
                        scorePreloadToPrep1,
                        new SleepAction(0.6),

                        //Pickup
                        intake.intakeFullOutPos(),
                        prep1ToPickup1,
                        new SleepAction(0.2),
                        intake.intakeWheelsOFF(),
                        intake.intakeHalfwayPos()

                )
        );

        // Transfer to Score 1
        Actions.runBlocking(
                new SequentialAction(
                        //Transfer
                        scoring.LS_BucketIntakePos(),
                        intake.intakeTransferPos(),
                        new SleepAction(0.6),
                        intake.intakeWheelsOUT(),
                        new SleepAction(1),

                        //Score
                        scoring.LS_TeleOpPos(),
                        scoring.LS_SAMPLEScorePos(),
                        new SleepAction(0.2),
                        pickup1ToScore1,
                        new SleepAction(0.2),
                        scoring.LS_BucketTipPos()
                )
        );
        // Prep Ground Pick 2 - Ground Pick 2
        Actions.runBlocking(
                new SequentialAction(
                        //Prep
                        scoring.LS_TeleOpPos(),
                        intake.intakeWheelsIN(),
                        intake.intakeHalfwayPos(),
                        new SleepAction(0.01),
                        score1ToPrep2,
                        new SleepAction(0.6),

                        //Pickup
                        intake.intakeFullOutPos(),
                        prep2ToPickup2,
                        new SleepAction(0.2),
                        intake.intakeWheelsOFF(),
                        intake.intakeHalfwayPos()
                )
        );

        // Transfer to Score 2
        Actions.runBlocking(
                new SequentialAction(
                        //Transfer
                        scoring.LS_BucketIntakePos(),
                        intake.intakeTransferPos(),
                        new SleepAction(0.6),
                        intake.intakeWheelsOUT(),
                        new SleepAction(1),

                        //Score
                        scoring.LS_TeleOpPos(),
                        scoring.LS_SAMPLEScorePos(),
                        new SleepAction(0.2),
                        pickup2ToScore2,
                        new SleepAction(0.2),
                        scoring.LS_BucketTipPos()
                )
        );
        // Prep Ground Pick 3 - Ground Pick 3
        Actions.runBlocking(
                new SequentialAction(
                        //Prep
                        scoring.LS_TeleOpPos(),
                        intake.intakeWheelsIN(),
                        intake.intakeHalfwayPos(),
                        new SleepAction(0.01),
                        score2ToPrep3,
                        new SleepAction(0.6),

                        //Pickup
                        intake.intakeFullOutPos(),
                        prep3ToPickup3,
                        new SleepAction(0.2),
                        intake.intakeWheelsOFF(),
                        intake.intakeHalfwayPos()
                )
        );

        // Transfer to Score 3
        Actions.runBlocking(
                new SequentialAction(
                        //Transfer
                        scoring.LS_BucketIntakePos(),
                        intake.intakeTransferPos(),
                        new SleepAction(0.6),
                        intake.intakeWheelsOUT(),
                        new SleepAction(1),

                        //Score
                        scoring.LS_TeleOpPos(),
                        scoring.LS_SAMPLEScorePos(),
                        new SleepAction(0.2),
                        pickup3ToScore3,
                        new SleepAction(0.2),
                        scoring.LS_BucketTipPos()
                )
        );





        // IF YES, run sub pick 1 automation
        if(sub1YESNO == 1){
            Actions.runBlocking(
                    new SequentialAction(
                            // Prep Sub Pick 1 - Sub Pick 1
                            scoring.LS_TeleOpPos(),
                            intake.intakeWheelsIN(),
                            new SleepAction(0.01),
                            score3ToPrepSub1,
                            intake.intakeHalfwayPos(),
                            new SleepAction(0.01),
                            prepSub1ToSubPick1,
                            intake.intakeFullOutPos(),
                            new SleepAction(0.5),

                            // Transfer to Mid Score 4
                            intake.intakeWheelsOFF(),
                            new SleepAction(0.01),
                            intake.intakeHalfwayPos(),
                            new SleepAction(0.01),
                            scoring.LS_BucketIntakePos(),
                            subPick1ToMidScore4,
                            intake.intakeTransferPos(),
                            new SleepAction(0.6),
                            intake.intakeWheelsOUT(),
                            new SleepAction(1),

                            // Mid Score 4 to Score 4
                            scoring.LS_TeleOpPos(),
                            intake.intakeHalfwayPos(),
                            intake.intakeWheelsIN(),
                            new SleepAction(0.01),
                            midScore4ToScore4,
                            scoring.LS_SAMPLEScorePos(),
                            new SleepAction(0.2),
                            scoring.LS_BucketTipPos()

                    )
            );
        }

        // IF YES, run sub pick 2 automation
        if(sub2YESNO == 1){
            Actions.runBlocking(
                    new SequentialAction(
                            // Prep Sub Pick 2 - Sub Pick 2
                            scoring.LS_TeleOpPos(),
                            intake.intakeWheelsIN(),
                            new SleepAction(0.01),
                            score4ToPrepSub2,
                            intake.intakeHalfwayPos(),
                            new SleepAction(0.01),
                            prepSub2ToSubPick2,
                            intake.intakeFullOutPos(),
                            new SleepAction(0.5),

                            // Transfer to Mid Score 5
                            intake.intakeWheelsOFF(),
                            new SleepAction(0.01),
                            intake.intakeHalfwayPos(),
                            new SleepAction(0.01),
                            scoring.LS_BucketIntakePos(),
                            subPick2ToMidScore5,
                            intake.intakeTransferPos(),
                            new SleepAction(0.6),
                            intake.intakeWheelsOUT(),
                            new SleepAction(1),

                            // Mid Score 5 to Score 5
                            scoring.LS_TeleOpPos(),
                            intake.intakeHalfwayPos(),
                            intake.intakeWheelsIN(),
                            new SleepAction(0.01),
                            midScore5ToScore5,
                            scoring.LS_SAMPLEScorePos(),
                            new SleepAction(0.2),
                            scoring.LS_BucketTipPos()
                    )
            );
        }

        // Park
        Actions.runBlocking(
                new ParallelAction(
                        scoring.LS_TeleOpPos(),
                        TrajectoryActionCloseOut
                )
        );


    }
}