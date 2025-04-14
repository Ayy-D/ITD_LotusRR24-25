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

@Config
@Autonomous(name = "SPEC -- RED", group = "Autonomous")
public class red_specimen extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();


    //Claw Components
    public class Claw {
        private Servo scC;
        public Claw(HardwareMap hardwareMap) {
            scC = hardwareMap.get(Servo.class, "scClaw"); //0.35 close | 0.9 open
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                scC.setPosition(0.26);
                return false;
            }
        }
        public Action closeClaw() {
            return new Claw.CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                scC.setPosition(1);
                return false;
            }
        }
        public Action openClaw() {
            return new Claw.OpenClaw();
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

        public class LS_SPECInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                scL.setPosition(0.12);
                scR.setPosition(0.12);
                scUD.setPosition(0.6);

                sL.setPower(0.85);
                sR.setPower(0.85);
                sL.setTargetPosition(5);
                sR.setTargetPosition(5);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
        public  Action LS_SPECInitPos() {
            return new LS_Scoring.LS_SPECInit();
        }

        public class LS_SPECBase implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                scR.setPosition(0.13);
                scL.setPosition(0.13);
                scUD.setPosition(0.92);


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
                time.reset();
                scR.setPosition(0.3);
                scL.setPosition(0.3);
                scUD.setPosition(0.97);

                sL.setPower(0.55);
                sL.setTargetPosition(610);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setPower(0.55);
                sR.setTargetPosition(610);
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
                time.reset();
                scR.setPosition(0.15);
                scL.setPosition(0.15);
                scUD.setPosition(1);

                sL.setPower(1);
                sR.setPower(1);
                sL.setTargetPosition(100);
                sR.setTargetPosition(100);
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

                sL.setPower(0.9);
                sL.setTargetPosition(1820);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setPower(0.9);
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
                scUD.setPosition(0.45);
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
        public Action LS_BucketTipPos() { return new LS_Scoring.LS_BucketTip(); }

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
            inTwist.setDirection(Servo.Direction.REVERSE);

        }

        public class IntakeInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(0);
                inL.setPower(0);
                inUD.setPosition(0.4);
                inArmL.setPosition(0.14);
                inArmR.setPosition(0.14);
                inTwist.setPosition(0.25);
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
                inUD.setPosition(0.425);
                inTwist.setPosition(0.25);

                return false;
            }
        }
        public Action intakeBasePos() { return new Intake.IntakeBase(); }

        public class IntakeHalfway implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.25);
                inArmL.setPosition(0.25);
                inUD.setPosition(0.65);
                inTwist.setPosition(0.42);

                return false;
            }
        }
        public Action intakeHalfwayPos() { return new Intake.IntakeHalfway(); }

        public class IntakeFullOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.34);
                inArmL.setPosition(0.34);
                inUD.setPosition(0.84);
                inTwist.setPosition(0.58);
                return false;
            }
        }
        public Action intakeFullOutPos() { return new Intake.IntakeFullOut(); }

        public class intakeHPCollapse implements Action{
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.14);
                inArmL.setPosition(0.14);
                inUD.setPosition(0.65);
                inTwist.setPosition(0.25);

                return false;
            }
        }
        public Action intakeHPCollapsePos() { return new Intake.intakeHPCollapse(); }

        public class IntakeTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.14);
                inArmL.setPosition(0.14);
                inUD.setPosition(0.2);
                inTwist.setPosition(0.35);

                return false;
            }

        }
        public Action intakeTransferPos() { return new Intake.IntakeTransfer(); }

        public class IntakeSweep implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.41);
                inArmL.setPosition(0.41);
                inUD.setPosition(0.85);
                inTwist.setPosition(0.58);
                return false;
            }
        }
        public Action intakeSweepPos() { return new Intake.IntakeSweep(); }

        public class IntakeWheelsIN implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(-1);
                inL.setPower(1);;
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
                inR.setPower(1);
                inL.setPower(-1);
                return false;
            }
        }
        public Action intakeWheelsOUT() { return new Intake.IntakeWheelsOUT();}
    }



    @Override
    public void runOpMode() {

        double[] xPose = {
                0, // initial pose — 0
                0, // scoring pose 1 — 1

                -35, // prep ground sweep 1 -- 2
                -35, // ground sweep 1 — 3

                -42, // prep ground sweep 2 — 4
                -42, // ground sweep 2 — 5

                -47, // prep ground sweep 3 — 6
                -44.5, // ground sweep 3 — 7

                -40, // first wall grab — 8
                -10, // first score — 9
                -40, // second wall grab — 10
                -6, // second score — 11
                -40, // third wall grab — 12
                -3, // third score — 13
                -40, // fourth wall grab — 14
                3 // third score — 15
        };

        double[] yPose = {
                61, // initial pose — 0
                37.5, // scoring pose 1 — 1

                36, // prep ground sweep 1 — 2
                56, // ground sweep 1 — 3

                36, // prep ground sweep 2 — 4
                53, // ground sweep 2 — 5

                22, // prep ground sweep 3 — 6
                50, // ground sweep 3 — 7

                60, // first wall grab — 8
                37.25, // first score — 9
                58, // second wall grab — 10
                37.5, // second score — 11
                58, // third wall grab — 12
                37.5, // third score — 13
                56.5, // fourth wall grab — 14
                37.5  // fourth score — 15
        };

        double[] angles = {
                Math.toRadians(270), // initial pose — 0
                Math.toRadians(270), // scoring pose — 1

                Math.toRadians(50), // prep ground sweep 1 — 2
                Math.toRadians(-15), // ground sweep 1 — 3

                Math.toRadians(50), // prep ground sweep 2 — 4
                Math.toRadians(-15), // ground sweep 2 — 5

                Math.toRadians(0), // prep ground sweep 3 — 6
                Math.toRadians(322.5), // ground sweep 3 — 7

                Math.toRadians(90), // first wall grab — 8
                Math.toRadians(-90), // first score — 9
                Math.toRadians(90), // second wall grab — 10
                Math.toRadians(-90), // second score — 11
                Math.toRadians(90), // third wall grab — 12
                Math.toRadians(-90), // third score — 13
                Math.toRadians(90), // fourth wall grab — 14
                Math.toRadians(-90) // fourth score — 15
        };

        Pose2d initialPose = new Pose2d(xPose[0], yPose[0], angles[0]);
        Pose2d scoringPose = new Pose2d(xPose[1], yPose[1], angles[1]);

        Pose2d prepToSweep1Pose = new Pose2d(xPose[2], yPose[2], angles[2]);
        Pose2d sweep1Pose = new Pose2d(xPose[3], yPose[3], angles[3]);

        Pose2d prepToSweep2Pose = new Pose2d(xPose[4], yPose[4], angles[4]);
        Pose2d sweep2Pose = new Pose2d(xPose[5], yPose[5], angles[5]);

        Pose2d prepToSweep3Pose = new Pose2d(xPose[6], yPose[6], angles[6]);
        Pose2d sweep3Pose = new Pose2d(xPose[7], yPose[7], angles[7]);

        Pose2d firstWallGrabPose = new Pose2d(xPose[8], yPose[8], angles[8]);
        Pose2d firstScorePose = new Pose2d(xPose[9], yPose[9], angles[9]);
        Pose2d secondWallGrabPose = new Pose2d(xPose[10], yPose[10], angles[10]);
        Pose2d secondScorePose = new Pose2d(xPose[11], yPose[11], angles[11]);
        Pose2d thirdWallGrabPose = new Pose2d(xPose[12], yPose[12], angles[12]);
        Pose2d thirdScorePose = new Pose2d(xPose[13], yPose[13], angles[13]);
        Pose2d fourthWallGrabPose = new Pose2d(xPose[14], yPose[14], angles[14]);
        Pose2d fourthScorePose = new Pose2d(xPose[15], yPose[15], angles[15]);



        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        LS_Scoring scoring = new LS_Scoring(hardwareMap);



        //Preload Specimen Score
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYConstantHeading(yPose[1]);

        //Prep to sweep floor spec 1
        TrajectoryActionBuilder tab2 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(xPose[2], yPose[2]), angles[2]);

        //Move to sweep floor spec 1
        TrajectoryActionBuilder tab3 = drive.actionBuilder(prepToSweep1Pose)
                .strafeToLinearHeading(new Vector2d(xPose[3], yPose[3]), angles[3]);

        //Prep to sweep floor spec 2
        TrajectoryActionBuilder tab4 = drive.actionBuilder(sweep1Pose)
                .strafeToLinearHeading(new Vector2d(xPose[4], yPose[4]), angles[4]);


        //Move to sweep floor spec 2
        TrajectoryActionBuilder tab5 = drive.actionBuilder(prepToSweep2Pose)
                .strafeToLinearHeading(new Vector2d(xPose[5], yPose[5]), angles[5]);

        //Prep to sweep floor spec 3
        TrajectoryActionBuilder tab6 = drive.actionBuilder(sweep2Pose)
                .strafeToLinearHeading(new Vector2d(xPose[6], yPose[6]), angles[6]);

        //Move to sweep floor spec 3
        TrajectoryActionBuilder tab7 = drive.actionBuilder(prepToSweep3Pose)
                .strafeToLinearHeading(new Vector2d(xPose[7], yPose[7]), angles[7]);


        //Prep to grab spec 1 from wall
        TrajectoryActionBuilder tab8 = drive.actionBuilder(sweep3Pose)
                .turnTo(angles[8])
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(xPose[8], yPose[8]), angles[8]);

        //Move to score spec 1 from wall
        TrajectoryActionBuilder tab9 = drive.actionBuilder(firstWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[9], yPose[9]), angles[9]);

        //Move to grab spec 2 from wall
        TrajectoryActionBuilder tab10 = drive.actionBuilder(firstScorePose)
                .strafeToLinearHeading(new Vector2d(-40, yPose[10]-3), Math.toRadians(89.99999))
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(xPose[10], yPose[10]), angles[10]);

        //Move to score spec 2 from wall
        TrajectoryActionBuilder tab11 = drive.actionBuilder(secondWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[11], yPose[11]), angles[11]);

        //Move to grab spec 3 from wall
        TrajectoryActionBuilder tab12 = drive.actionBuilder(secondScorePose)
                .strafeToLinearHeading(new Vector2d(-40, yPose[12]-3), Math.toRadians(89.99999))
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(xPose[12], yPose[12]), angles[12]);


        //Move to score spec 2 from wall
        TrajectoryActionBuilder tab13 = drive.actionBuilder(thirdWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[13], yPose[13]), angles[13]);

        //Move to grab spec 3 from wall
        TrajectoryActionBuilder tab14 = drive.actionBuilder(thirdScorePose)
                .strafeToLinearHeading(new Vector2d(-40, yPose[14]-3), Math.toRadians(89.99999))
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(xPose[14], yPose[14]), angles[14]);


        //Move to score spec 2 from wall
        TrajectoryActionBuilder tab15 = drive.actionBuilder(fourthWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[15], yPose[15]), angles[15]);


        //Park
        Action TrajectoryActionPark = drive.actionBuilder(fourthScorePose)
                .strafeToLinearHeading(new Vector2d(xPose[12], yPose[12]), Math.toRadians(330))
                .afterTime(0.5, intake.intakeFullOutPos())
                .build();



        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(intake.intakeInitPos());

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        // Wait for the start signal
        waitForStart();
        if (isStopRequested()) return;
        time.reset();



        Action initToScoreTrajectory = tab1.build();

        Action initScoreToPrepSweep1 = tab2.build();
        Action prepSweep1toSweep1 = tab3.build();

        Action sweep1toPrepSweep2 = tab4.build();
        Action prepSweep2toSweep2 = tab5.build();

        Action sweep2toPrepSweep3 = tab6.build();
        Action prepSweep3toSweep3 = tab7.build();

        Action sweep3ToWall1 = tab8.build();
        Action wall1ToScore1 = tab9.build();

        Action score1ToWall2 = tab10.build();
        Action wall2toScore2 = tab11.build();

        Action score2ToWall3 = tab12.build();
        Action wall3ToScore3 = tab13.build();

        Action score3ToWall4 = tab14.build();
        Action wall4ToScore4 = tab15.build();


        // Preload Spec Score
        Actions.runBlocking(
                new SequentialAction(
                        scoring.LS_SPECScorePos(),
                        new SleepAction(0.01),
                        initToScoreTrajectory,
                        scoring.LS_SPECPullPos(),
                        new SleepAction(0.2),
                        claw.openClaw(),
                        new SleepAction(0.01),
                        intake.intakeHalfwayPos()

                        )
        );

        // Prep Ground Sweep 1
        Actions.runBlocking(
                new SequentialAction(
                        scoring.LS_SPECBasePos(),
                        initScoreToPrepSweep1,
                        new SleepAction(0.01),
                        intake.intakeSweepPos(),
                        new SleepAction(0.3)
                )
        );

        // Ground Sweep 1
        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeWheelsOUT(),
                        prepSweep1toSweep1,
                        new SleepAction(0.01),
                        intake.intakeHalfwayPos()
                )
        );

        // Prep Ground Sweep 2
        Actions.runBlocking(
                new SequentialAction(
                        sweep1toPrepSweep2,
                        new SleepAction(0.01),
                        intake.intakeSweepPos(),
                        new SleepAction(0.3)
                )
        );

        // Ground Sweep 2
        Actions.runBlocking(
                new SequentialAction(
                        prepSweep2toSweep2,
                        new SleepAction(0.01),
                        intake.intakeHalfwayPos()
                )
        );

        // Prep Ground Sweep 3
        Actions.runBlocking(
                new SequentialAction(
                        sweep2toPrepSweep3,
                        new SleepAction(0.01),
                        intake.intakeSweepPos(),
                        new SleepAction(0.3)
                )
        );

        // Ground Sweep 2
        Actions.runBlocking(
                new SequentialAction(
                        prepSweep3toSweep3,
                        new SleepAction(0.01),
                        intake.intakeBasePos()
                )
        );


        // Ground Sweep 3
        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeWheelsIN(),
                        prepSweep1toSweep1,
                        new SleepAction(0.01),
                        intake.intakeBasePos()
                )
        );


        // Specimen Scoring
        Actions.runBlocking(
                new SequentialAction(

                        // Specimen 1
                        sweep3ToWall1,
                        new SleepAction(0.1),
                        claw.closeClaw(),
                        new SleepAction(0.2),
                        scoring.LS_SPECScorePos(),
                        wall1ToScore1,
                        scoring.LS_SPECPullPos(),
                        new SleepAction(0.2),
                        claw.openClaw(),
                        scoring.LS_SPECBasePos(),


                        // Specimen 2
                        score1ToWall2,
                        new SleepAction(0.1),
                        claw.closeClaw(),
                        new SleepAction(0.2),
                        scoring.LS_SPECScorePos(),
                        wall2toScore2,
                        scoring.LS_SPECPullPos(),
                        new SleepAction(0.2),
                        claw.openClaw(),
                        scoring.LS_SPECBasePos(),


                        // Specimen 3
                        score2ToWall3,
                        new SleepAction(0.1),
                        claw.closeClaw(),
                        new SleepAction(0.2),
                        scoring.LS_SPECScorePos(),
                        wall3ToScore3,
                        scoring.LS_SPECPullPos(),
                        new SleepAction(0.2),
                        scoring.LS_SPECBasePos(),
                        claw.openClaw(),

                        // Specimen 4
                        score3ToWall4,
                        new SleepAction(0.1),
                        claw.closeClaw(),
                        new SleepAction(0.2),
                        scoring.LS_SPECScorePos(),
                        wall4ToScore4,
                        scoring.LS_SPECPullPos(),
                        new SleepAction(0.2),
                        scoring.LS_SPECBasePos(),
                        claw.openClaw()

                )
        );

        // Park
        Actions.runBlocking(
                TrajectoryActionPark
        );




    }
}