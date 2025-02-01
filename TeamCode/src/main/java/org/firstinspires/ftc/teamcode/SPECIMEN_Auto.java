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
@Autonomous(name = "SPECIMEN Auto", group = "Autonomous")
public class SPECIMEN_Auto extends LinearOpMode {
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
                rotL.setPower(0.8);
                rotR.setPower(0.8);
                rotL.setTargetPosition(0);
                rotR.setTargetPosition(0);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }
        public Action rotationBasePos() {
            return new rotation.rotationBase();
        }

        public class rotationUP implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotL.setPower(0.8);
                rotR.setPower(0.8);
                rotL.setTargetPosition(100);
                rotR.setTargetPosition(100);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            return new LS_SPECInit();
        }

        public class LS_SPECBase implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                scL.setPosition(0.16);
                scR.setPosition(0.16);
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
            return new LS_SPECBase();
        }

        public class LS_SPECScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                time.reset();
                scR.setPosition(0.3);
                scL.setPosition(0.3);
                scUD.setPosition(0.97);

                sL.setPower(0.9);
                sL.setTargetPosition(650);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setPower(0.9);
                sR.setTargetPosition(650);
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

                sL.setPower(0.8);
                sR.setPower(0.8);
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
                inUD.setPosition(0.65);
                inTwist.setPosition(0.35);

                return false;
            }
        }
        public Action intakeHalfwayPos() { return new Intake.IntakeHalfway(); }

        public class IntakeFullOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.33);
                inArmL.setPosition(0.3);
                inUD.setPosition(0.88);
                inTwist.setPosition(0.56);

                return false;
            }
        }
        public Action intakeFullOutPos() { return new Intake.IntakeFullOut(); }

        public class IntakeTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.12);
                inArmL.setPosition(0.12);
                inUD.setPosition(0.2);
                inTwist.setPosition(0.35);

                return false;
            }

        }
        public Action intakeTransferPos() { return new Intake.IntakeTransfer(); }

        public class IntakeMiniBase implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.14);
                inArmL.setPosition(0.14);
                inUD.setPosition(0.75);
                inTwist.setPosition(0.35);
                return false;
            }
        }
        public Action intakeMiniBasePos() { return new Intake.IntakeMiniBase(); }

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
                0, // initial pose
                0, // scoring pose 1

                -31, // prep ground pick 1
                -34, // ground pick 1
                -43, // human drop 1

                -43, // prep ground pick 2
                -48, // ground pick 2
                -48, // human drop 2

                -48, // prep ground pick 3
                -52, // ground pick 3
                -48, // human drop 3

                -40, // first wall grab
                -2, // first score
                -40, // second wall grab
                -3, // second score
                -40, // third wall grab
                -4 // third score
        };

        double[] yPose = {
                60, // initial pose
                39.25, // scoring pose 1

                45, // prep ground pick 1
                42, // ground pick 1
                48, // human drop 1

                45, // prep ground pick 2
                41, // ground pick 2
                48, // human drop 2

                44, // prep ground pick 3
                42, // ground pick 3
                48, // human drop 3

                55, // first wall grab
                38.75, // first score
                55.5, // second wall grab
                39, // second score
                55.5, // third wall grab
                39 // third score
        };

        double[] angles = {
                Math.toRadians(270), // initial pose
                Math.toRadians(270), // scoring pose

                Math.toRadians(50), // prep ground pick 1
                Math.toRadians(50), // ground pick 1
                Math.toRadians(315), // human drop 1

                Math.toRadians(55), // prep ground pick 2
                Math.toRadians(55), // ground pick 2
                Math.toRadians(315), // human drop 2

                Math.toRadians(45), // prep ground pick 3
                Math.toRadians(45), // ground pick 3
                Math.toRadians(315), // human drop 3

                Math.toRadians(90), // first wall grab
                Math.toRadians(-90), // first score
                Math.toRadians(92), // second wall grab
                Math.toRadians(-90), // second score
                Math.toRadians(92), // third wall grab
                Math.toRadians(-90), // third score
        };

        Pose2d initialPose = new Pose2d(xPose[0], yPose[0], angles[0]);
        Pose2d scoringPose = new Pose2d(xPose[1], yPose[1], angles[1]);

        Pose2d prepToPick1Pose = new Pose2d(xPose[2], yPose[2], angles[2]);
        Pose2d pick1Pose = new Pose2d(xPose[3], yPose[3], angles[3]);
        Pose2d human1Pose = new Pose2d(xPose[4], yPose[4], angles[4]);

        Pose2d prepToPick2Pose = new Pose2d(xPose[5], yPose[5], angles[5]);
        Pose2d pick2Pose = new Pose2d(xPose[6], yPose[6], angles[6]);
        Pose2d human2Pose = new Pose2d(xPose[7], yPose[7], angles[7]);

        Pose2d prepToPick3Pose = new Pose2d(xPose[8], yPose[8], angles[8]);
        Pose2d pick3Pose = new Pose2d(xPose[9], yPose[9], angles[9]);
        Pose2d human3Pose = new Pose2d(xPose[10], yPose[10], angles[10]);


        Pose2d firstWallGrabPose = new Pose2d(xPose[11], yPose[11], angles[11]);
        Pose2d firstScorePose = new Pose2d(xPose[12], yPose[12], angles[12]);
        Pose2d secondWallGrabPose = new Pose2d(xPose[13], yPose[13], angles[13]);
        Pose2d secondScorePose = new Pose2d(xPose[14], yPose[14], angles[14]);
        Pose2d thirdWallGrabPose = new Pose2d(xPose[15], yPose[15], angles[15]);
        Pose2d thirdScorePose = new Pose2d(xPose[16], yPose[16], angles[16]);


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        LS_Scoring scoring = new LS_Scoring(hardwareMap);
        rotation rotation = new rotation(hardwareMap);



        //Preload Specimen Score
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYConstantHeading(yPose[1]);


        //Prep to grab floor spec 1
        TrajectoryActionBuilder tab2 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(xPose[2], yPose[2]), angles[2]);

        //Move to grab floor spec 1
        TrajectoryActionBuilder tab3 = drive.actionBuilder(prepToPick1Pose)
                .strafeToLinearHeading(new Vector2d(xPose[3], yPose[3]), angles[3]);

        //Move to drop floor spec 1
        TrajectoryActionBuilder tab4 = drive.actionBuilder(pick1Pose)
                .strafeToLinearHeading(new Vector2d(xPose[4], yPose[4]), angles[4]);


        //Prep to grab floor spec 2
        TrajectoryActionBuilder tab5 = drive.actionBuilder(human1Pose)
                .strafeToLinearHeading(new Vector2d(xPose[5], yPose[5]), angles[5]);

        //Move to grab floor spec 2
        TrajectoryActionBuilder tab6 = drive.actionBuilder(prepToPick2Pose)
                .strafeToLinearHeading(new Vector2d(xPose[6], yPose[6]), angles[6]);

        //Move to drop floor spec 2
        TrajectoryActionBuilder tab7 = drive.actionBuilder(pick2Pose)
                .strafeToLinearHeading(new Vector2d(xPose[7], yPose[7]), angles[7]);


        //Prep to grab floor spec 3
        TrajectoryActionBuilder tab8 = drive.actionBuilder(human2Pose)
                .strafeToLinearHeading(new Vector2d(xPose[8], yPose[8]), angles[8]);

        //Move to grab floor spec 3
        TrajectoryActionBuilder tab9 = drive.actionBuilder(prepToPick3Pose)
                .strafeToLinearHeading(new Vector2d(xPose[9], yPose[9]), angles[9]);

        //Move to drop floor spec 3
        TrajectoryActionBuilder tab10 = drive.actionBuilder(pick3Pose)
                .strafeToLinearHeading(new Vector2d(xPose[10], yPose[10]), angles[10]);



        //Move to grab spec 1 from wall
        TrajectoryActionBuilder tab11 = drive.actionBuilder(human3Pose)
                .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(90))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(xPose[11], yPose[11]), angles[11]);

        //Move to score specimen 1 from wall
        TrajectoryActionBuilder tab12 = drive.actionBuilder(firstWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[12], yPose[12]), angles[12]);


        //Move to wall grab spec 2
        TrajectoryActionBuilder tab13 = drive.actionBuilder(firstScorePose)
                .strafeToLinearHeading(new Vector2d(-40, 55), Math.toRadians(89.99999))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(xPose[13], yPose[13]), angles[13]);

        //Move to score specimen 2 from wall
        TrajectoryActionBuilder tab14 = drive.actionBuilder(secondWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[14], yPose[14]), angles[14]);

        //Move to wall grab spec 2
        TrajectoryActionBuilder tab15 = drive.actionBuilder(secondScorePose)
                .strafeToLinearHeading(new Vector2d(-40, 55), Math.toRadians(89.99999))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(xPose[15], yPose[15]), angles[15]);

        //Move to score specimen 2 from wall
        TrajectoryActionBuilder tab16 = drive.actionBuilder(thirdWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[16], yPose[16]), angles[16]);

        //Park
        Action TrajectoryActionPark = drive.actionBuilder(thirdScorePose)
                .strafeToLinearHeading(new Vector2d(-34, 46), Math.toRadians(315))
                .build();



        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(rotation.rotationBasePos());
        Actions.runBlocking(scoring.LS_SPECInitPos());

         while (!isStopRequested() && !opModeIsActive()) {
         telemetry.update();
         }

        // Wait for the start signal
        waitForStart();
        if (isStopRequested()) return;
        time.reset();


        Action initToScoreTrajectory = tab1.build();

        Action prepToPick1 = tab2.build();
        Action scoreToPick1 = tab3.build();
        Action pick1ToHuman1 = tab4.build();

        Action human1toPrep2 = tab5.build();
        Action prep2ToPick2 = tab6.build();
        Action pick2ToHuman2 = tab7.build();

        Action human2ToPrep3 = tab8.build();
        Action prep3ToPick3 = tab9.build();
        Action pick3ToHuman3 = tab10.build();
        Action human3ToWall = tab11.build();

        Action wallToScore1 = tab12.build();

        Action scoreToHumanTrajectory2 = tab13.build();
        Action humanToScoreTrajectory2 = tab14.build();

        Action scoreToHumanTrajectory3 = tab15.build();
        Action humanToScoreTrajectory3 = tab16.build();


        // Preload Spec Score
        Actions.runBlocking(
                new SequentialAction(
                        rotation.rotationUPPos(),
                        new SleepAction(0.5),
                        scoring.LS_SPECScorePos(),
                        new SleepAction(0.01),
                        intake.intakeHalfwayPos(),
                        new SleepAction(0.01),
                        initToScoreTrajectory,
                        scoring.LS_SPECPullPos(),
                        new SleepAction(0.4),
                        claw.openClaw()
                )
        );

        // Prep Ground Pick 1
        Actions.runBlocking(
                new ParallelAction(
                        scoring.LS_SPECBasePos(),
                        prepToPick1,
                        intake.intakeWheelsIN(),

                        new SleepAction(0.01)
                )
        );

        // Ground Pick 1
        Actions.runBlocking(
                new ParallelAction(
                        intake.intakeFullOutPos(),
                        new SleepAction(0.01),
                        scoreToPick1,
                        new SleepAction(0.4)
                )
        );


        // Drop Ground Pick 1
        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeWheelsOFF(),
                        new SleepAction(0.01),
                        intake.intakeHalfwayPos(),
                        pick1ToHuman1,
                        intake.intakeWheelsOUT(),
                        new SleepAction(0.65)
                )
        );


        // Prep Ground Pick 2
        Actions.runBlocking(
                new ParallelAction(
                        human1toPrep2,
                        intake.intakeWheelsIN()
                )
        );

        // Ground Pick 2
        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeFullOutPos(),
                        new SleepAction(0.2),
                        prep2ToPick2,
                        new SleepAction(0.4)
                )
        );

        // Drop Ground Pick 2
        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeWheelsOFF(),
                        new SleepAction(0.01),
                        intake.intakeHalfwayPos(),
                        pick2ToHuman2,
                        intake.intakeWheelsOUT(),
                        new SleepAction(0.65)
                )
        );

        // Prep Ground Pick 3
        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeWheelsIN(),
                        human2ToPrep3,
                        new SleepAction(0.01)
                )
        );


        // Ground Pick 3
        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeFullOutPos(),
                        new SleepAction(0.1),
                        prep3ToPick3,
                        new SleepAction(0.4)

                )
        );

        // Drop Ground Pick 3
        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeWheelsOFF(),
                        new SleepAction(0.01),
                        intake.intakeMiniBasePos(),
                        pick3ToHuman3,
                        intake.intakeWheelsOUT(),
                        new SleepAction(0.65),
                        intake.intakeWheelsOFF()

                        )
        );

        // Specimen Scoring
        Actions.runBlocking(
                new SequentialAction(

                        // Specimen 1
                        human3ToWall,
                        new SleepAction(0.2),
                        claw.closeClaw(),
                        new SleepAction(0.1),
                        scoring.LS_SPECScorePos(),
                        wallToScore1,
                        scoring.LS_SPECPullPos(),
                        new SleepAction(0.3),
                        claw.openClaw(),
                        scoring.LS_SPECBasePos(),


                        // Specimen 2
                        scoreToHumanTrajectory2,
                        new SleepAction(0.2),
                        claw.closeClaw(),
                        new SleepAction(0.1),
                        scoring.LS_SPECScorePos(),
                        humanToScoreTrajectory2,
                        scoring.LS_SPECPullPos(),
                        new SleepAction(0.3),
                        claw.openClaw(),
                        scoring.LS_SPECBasePos(),

                        /*
                        // Specimen 3
                        scoreToHumanTrajectory3,
                        new SleepAction(0.25),
                        claw.closeClaw(),
                        new SleepAction(0.25),
                        scoring.LS_SPECScorePos(),
                        humanToScoreTrajectory3,
                        new SleepAction(0.1),
                        scoring.LS_SPECPullPos(),
                        new SleepAction(0.2),
                        scoring.LS_SPECBasePos(),
                        claw.closeClaw(),

                         */

                        // Park
                        intake.intakeFullOutPos(),
                        TrajectoryActionPark

                )
        );




    }
}
