package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
                scC.setPosition(0.35);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                scC.setPosition(0.9);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
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

        private DcMotorEx rotR;
        private DcMotorEx rotL;

        public LS_Scoring(HardwareMap hardwareMap) {
            scL = hardwareMap.get(Servo.class, "scArmL"); //0.95 goes toward intake, 0 goes outward from robot
            scR = hardwareMap.get(Servo.class, "scArmR"); //0.95 goes toward intake, 0 goes outward from robot
            scUD = hardwareMap.get(Servo.class, "scUD"); //1 is the position for depositing an element, 0.8 for intake, <0.8 to keep it up
            scUD.setDirection(Servo.Direction.REVERSE);
            scC = hardwareMap.get(Servo.class, "scClaw"); //0.35 close, 0.9 open)

            sL = hardwareMap.get(DcMotorEx.class, "slideL");
            sR = hardwareMap.get(DcMotorEx.class, "slideR");

            rotR = hardwareMap.get(DcMotorEx.class, "rotateR");
            rotL = hardwareMap.get(DcMotorEx.class, "rotateL");

            //RUN Encoders
            rotR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //ROTATION BRAKE Behavior
            rotR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rotL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //ROTATION Set Direction
            rotR.setDirection(DcMotorSimple.Direction.REVERSE);
            rotL.setDirection(DcMotorSimple.Direction.FORWARD);

            //RESET Encoders
            sL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //SLIDE BRAKE Behavior
            sR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //SLIDE Set Direction
            sL.setDirection(DcMotorSimple.Direction.FORWARD);
            sR.setDirection(DcMotorSimple.Direction.REVERSE);


        }

        public class LS_ArmBasePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                scL.setPosition(0.12);
                scR.setPosition(0.12);
                scUD.setPosition(0.85);

                sL.setPower(0.85);
                sR.setPower(0.85);
                sL.setTargetPosition(15);
                sR.setTargetPosition(15);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rotL.setPower(0.4);
                rotR.setPower(0.4);
                rotL.setTargetPosition(200);
                rotR.setTargetPosition(200);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }
        public  Action LS_ArmBase() {
            return new LS_ArmBasePos();
        }

        public class LS_ArmScorePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                scL.setPosition(0.6); //verify
                scR.setPosition(0.6);
                scUD.setPosition(1); //verify

                sL.setPower(0.8);
                sR.setPower(0.8);
                sL.setTargetPosition(845);
                sR.setTargetPosition(845);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                rotL.setPower(0.4);
                rotR.setPower(0.4);
                rotL.setTargetPosition(200);
                rotR.setTargetPosition(200);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }
        public Action LS_ArmScore() {
            return new LS_ArmScorePos();
        }

        public class LS_ArmScorePull implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                scL.setPosition(0.6); //verify
                scR.setPosition(0.6);
                scUD.setPosition(1); //verify

                sL.setPower(0.8);
                sR.setPower(0.8);
                sL.setTargetPosition(700);
                sR.setTargetPosition(700);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rotL.setPower(0.4);
                rotR.setPower(0.4);
                rotL.setTargetPosition(200);
                rotR.setTargetPosition(200);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }
        public Action LS_ArmScorePull() {
            return new LS_ArmScorePos();
        }

        public class LS_Arm_TeleOpPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotL.setPower(0.4);
                rotL.setTargetPosition(200);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setPower(0.4);
                rotR.setTargetPosition(200);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                sL.setPower(0.9);
                sL.setTargetPosition(15);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setPower(0.9);
                sR.setTargetPosition(15);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                scR.setPosition(0.4);
                scL.setPosition(0.4);
                scUD.setPosition(0.9);
                scC.setPosition(0.35);

                return false;
            }
        }
        public Action LS_Arm_TeleOpPrep() { return new LS_Arm_TeleOpPos(); }
    }


    //Intake components
    public class IntakeHold {
        private CRServo inR;
        private CRServo inL;
        private Servo inUD;
        private Servo inArmR;
        private Servo inArmL;
        private Servo inTwist;

        public IntakeHold(HardwareMap hardwareMap) {
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
        public Action intakeInit() {
            return new IntakeInit();
        }


        public class IntakeBasePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmL.setPosition(0.14);
                inArmR.setPosition(0.14);
                inUD.setPosition(0.65);
                inTwist.setPosition(0.35);

                return false;
            }
        }
        public Action intakeBasePos() { return new IntakeBasePos(); }

        public class IntakeHalfwayPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.25);
                inArmL.setPosition(0.25);
                inUD.setPosition(0.65);
                inTwist.setPosition(0.35);

                return false;
            }
        }
        public Action intakeHalfwayPos() { return new IntakeHalfwayPos(); }

        public class IntakeFullOutPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.33);
                inArmL.setPosition(0.3);
                inUD.setPosition(0.88);
                inTwist.setPosition(0.56);

                return false;
            }
        }
        public Action intakeFullOutPos() { return new IntakeFullOutPos(); }


        public class IntakeWheelsIN implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(-1);
                inL.setPower(1);;
                return false;
            }
        }
        public Action intakeWheelsIN() { return new IntakeWheelsIN(); }


        public class IntakeWheelsOFF implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(0);
                inL.setPower(0);
                return false;
            }
        }
        public Action intakeWheelsOFF() { return new IntakeWheelsOFF(); }


        public class IntakeWheelsOUT implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(1);
                inL.setPower(-1);
                return false;
            }
        }
        public Action intakeWheelsOUT() { return new IntakeWheelsOUT(); }
    }


    @Override
    public void runOpMode() {

        double[] xPose = {
                0, // initial pose
                0, // scoring pose 1

                -34, // ground pick 1
                -36, // human drop 1
                -44, // ground pick 2
                -48, // human drop 2
                -54, // ground pick 3
                -54, // human drop 3

                -40, // first wall grab
                0, // first score
                -40, // second wall grab
                0, // second score
                -40, // third wall grab
                0 // third score
        };

        double[] yPose = {
                60, // initial pose
                34.5, // scoring pose 1

                42, // ground pick 1
                48, // human drop 1
                42, // ground pick 2
                48, // human drop 2
                42, // ground pick 3
                48, // human drop 3

                58, // first wall grab
                33, // first score
                58.5, // second wall grab
                33, // second score
                58.5, // third wall grab
                33 // third score
        };

        double[] angles = {
                Math.toRadians(270), // initial pose
                Math.toRadians(270), // scoring pose

                Math.toRadians(45), // ground pick 1
                Math.toRadians(315), // human drop 1
                Math.toRadians(45), // ground pick 2
                Math.toRadians(315), // human drop 2
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

        Pose2d pick1Pose = new Pose2d(xPose[2], yPose[2], angles[2]);
        Pose2d human1Pose = new Pose2d(xPose[3], yPose[3], angles[3]);
        Pose2d pick2Pose = new Pose2d(xPose[4], yPose[4], angles[4]);
        Pose2d human2Pose = new Pose2d(xPose[5], yPose[5], angles[5]);
        Pose2d pick3Pose = new Pose2d(xPose[6], yPose[6], angles[6]);
        Pose2d human3Pose = new Pose2d(xPose[7], yPose[7], angles[7]);


        Pose2d firstWallGrabPose = new Pose2d(xPose[8], yPose[8], angles[8]);
        Pose2d firstScorePose = new Pose2d(xPose[9], yPose[9], angles[9]);
        Pose2d secondWallGrabPose = new Pose2d(xPose[10], yPose[10], angles[10]);
        Pose2d secondScorePose = new Pose2d(xPose[11], yPose[11], angles[11]);
        Pose2d thirdWallGrabPose = new Pose2d(xPose[12], yPose[12], angles[12]);
        Pose2d thirdScorePose = new Pose2d(xPose[13], yPose[13], angles[13]);


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Claw claw = new Claw(hardwareMap);
        IntakeHold intake = new IntakeHold(hardwareMap);
        LS_Scoring scoring = new LS_Scoring(hardwareMap);



        //Preload Specimen Score
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYConstantHeading(yPose[1])
                .waitSeconds(0.5);

        //Move to grab floor spec 1
        TrajectoryActionBuilder tab2 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(xPose[2], yPose[2]), angles[2])
                .waitSeconds(0.5);

        //Move to drop floor spec 1
        TrajectoryActionBuilder tab3 = drive.actionBuilder(pick1Pose)
                .strafeToLinearHeading(new Vector2d(xPose[3], yPose[3]), angles[3])
                .waitSeconds(0.25);

        //Move to grab floor spec 2
        TrajectoryActionBuilder tab4 = drive.actionBuilder(human1Pose)
                .strafeToLinearHeading(new Vector2d(xPose[4], yPose[4]), angles[4])
                .waitSeconds(0.5);

        //Move to drop floor spec 2
        TrajectoryActionBuilder tab5 = drive.actionBuilder(pick2Pose)
                .strafeToLinearHeading(new Vector2d(xPose[5], yPose[5]), angles[5])
                .waitSeconds(0.25);

        //Move to grab floor spec 3
        TrajectoryActionBuilder tab6 = drive.actionBuilder(human2Pose)
                .strafeToLinearHeading(new Vector2d(xPose[6], yPose[6]), angles[6])
                .waitSeconds(0.5);

        //Move to drop floor spec 3
        TrajectoryActionBuilder tab7 = drive.actionBuilder(pick3Pose)
                .strafeToLinearHeading(new Vector2d(xPose[7], yPose[7]), angles[7])
                .waitSeconds(0.25);

        //Move to grab spec 1 from wall
        TrajectoryActionBuilder tab8 = drive.actionBuilder(human3Pose)
                .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(90))
                    .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(xPose[8], yPose[8]), angles[8])
                    .waitSeconds(0.25);

        //Move to score specimen 1 from wall
        TrajectoryActionBuilder tab9 = drive.actionBuilder(firstWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[9], yPose[9]), angles[9])
                    .waitSeconds(0.5);


        //Move to wall grab spec 2
        TrajectoryActionBuilder tab10 = drive.actionBuilder(firstScorePose)
                .strafeToLinearHeading(new Vector2d(-40, 55), Math.toRadians(89.99999))
                    .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(xPose[10], yPose[10]), angles[10])
                    .waitSeconds(0.25);

        //Move to score specimen 2 from wall
        TrajectoryActionBuilder tab11 = drive.actionBuilder(secondWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[11], yPose[11]), angles[11])
                    .waitSeconds(0.5);

        //Move to wall grab spec 2
        TrajectoryActionBuilder tab12 = drive.actionBuilder(secondScorePose)
                .strafeToLinearHeading(new Vector2d(-40, 55), Math.toRadians(89.99999))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(xPose[12], yPose[12]), angles[12])
                .waitSeconds(0.25);

        //Move to score specimen 2 from wall
        TrajectoryActionBuilder tab13 = drive.actionBuilder(thirdWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[13], yPose[13]), angles[13])
                .waitSeconds(0.5);


        //Park
        Action TrajectoryActionPark = drive.actionBuilder(thirdScorePose)
                .strafeToLinearHeading(new Vector2d(-28, 40), Math.toRadians(315))
                .build();



        //Actions.runBlocking(claw.closeClaw());
        //Actions.runBlocking(intake.intakeInit());
        /***
         while (!isStopRequested() && !opModeIsActive()) {
         telemetry.update();
         }
         ***/
        // Wait for the start signal
        waitForStart();
        if (isStopRequested()) return;
        time.reset();


        Action initToScoreTrajectory = tab1.build();

        Action scoreToPick1 = tab2.build();

        Action pick1ToHuman1 = tab3.build();
        Action human1toPick2 = tab4.build();

        Action pick2ToHuman2 = tab5.build();
        Action human2ToPick3 = tab6.build();

        Action pick3ToHuman3 = tab7.build();
        Action human3ToWall = tab8.build();

        Action wallToScore1 = tab9.build();

        Action scoreToHumanTrajectory2 = tab10.build();
        Action humanToScoreTrajectory2 = tab11.build();

        Action scoreToHumanTrajectory3 = tab12.build();
        Action humanToScoreTrajectory3 = tab13.build();



        Actions.runBlocking(
                new SequentialAction(
                        initToScoreTrajectory, //runs

                        scoreToPick1, //runs
                        pick1ToHuman1, //runs
                        human1toPick2, //runs
                        pick2ToHuman2, //runs
                        human2ToPick3, //runs
                        pick3ToHuman3, //runs

                        human3ToWall, // move to first wall spec
                        wallToScore1, // score first wall spec

                        scoreToHumanTrajectory2, // move to second wall spec
                        humanToScoreTrajectory2, // score second wall spec
                        scoreToHumanTrajectory3, // move to third wall spec
                        humanToScoreTrajectory3, // score third wall spec
                        TrajectoryActionPark


                        /*
                        //          SCORE PRELOAD SPECIMEN
                        // PARALLEL Slide & Arm into Scoring Position, Move Intake to Base Pos, initToScoreTrajectory,
                        new ParallelAction(
                                //scoring.LS_ArmScore(),
                                //intake.intakeBasePos(),
                                initToScoreTrajectory

                        ),



                        //          MOVE AND PREP TO FLOOR/WALL GRABS
                        // Score Preload Specimen, delay, Claw Open
                        //scoring.LS_ArmScorePull(),
                        //insert 0.125s delay here
                        //claw.openClaw(),


                        // PARALLEL Move Preload Score to First Pickup, Move Intake to Halfway, Slide/Arm Down to Intake Position
                        new ParallelAction(
                                //intake.intakeHalfwayPos(),
                                //scoring.LS_ArmBase(),
                                scoreToPick1

                        ),

                        //          FLOOR GRAB #1
                        // PARALLEL Extend Intake #1, Intake Wheels IN, Wait for Intake to Grab
                        //new ParallelAction(

                        //),

                        // PARALLEL Stop Intake Wheels, Move Intake to Halfway, Floor to Human Trajectory #1
                        new ParallelAction(
                                pick1ToHuman1

                        ),

                        // PARALLEL Intake wheels OUT, Slight delay, Human to pickup 2
                        new ParallelAction(
                                human1toPick2
                        ),


                        //          FLOOR GRAB #2
                        // PARALLEL Extend Intake #2, Intake Wheels IN, Wait for Intake to Grab
                        //new ParallelAction(

                        //),

                        // PARALLEL Stop Intake Wheels, Move Intake to Halfway, Floor to Human Trajectory #2
                        new ParallelAction(
                                pick2ToHuman2
                        ),

                        // PARALLEL Intake wheels OUT, Slight Delay, Human to Pickup 3
                        new ParallelAction(
                                human2ToPick3
                        ),


                        //          FLOOR GRAB #3
                        // PARALLEL Extend Intake #3, Intake Wheels IN, Wait for Intake to Grab
                        //new ParallelAction(

                        //),

                        // PARALLEL Stop Intake Wheels, Move Intake to Halfway, Floor to Human Trajectory #3
                        new ParallelAction(
                                pick3ToHuman3
                        ),



                        //          MOVE TO WALL GRAB
                        // PARALLEL Intake Wheels OUT, Slight Delay, Move Intake to Base Pos, Move to Human
                        new ParallelAction(
                                human3ToWall
                        ),



                        //          WALL SPECIMEN #1
                        // Claw close

                        // PARALLEL Slide/Arm into Scoring Position, Move Intake to Base Pos, humanToScoreTrajectory #1,
                        new ParallelAction(
                                humanToScoreTrajectory
                        ),


                        // Score Specimen, delay, Claw Open

                        // PARALLEL Slide/Arm into Intake Position, scoreToHumanTrajectory #1
                        new ParallelAction(
                                scoreToHumanTrajectory
                        ),



                        //          WALL SPECIMEN #2
                        // Claw Close

                        // PARALLEL Slide/Arm into Scoring Position, Move Intake to Base Pos, humanToScoreTrajectory #2
                        new ParallelAction(
                                humanToScoreTrajectory
                        ),


                        // Score Specimen, delay, Claw Open

                        // PARALLEL Slide/Arm into Intake Position, scoreToHumanTrajectory #2
                        new ParallelAction(
                                scoreToHumanTrajectory
                        ),



                        //          WALL SPECIMEN #3
                        // Claw Close

                        // PARALLEL Slide/Arm into Scoring Position, Move Intake to Base Pos, humanToScoreTrajectory #3,
                        new ParallelAction(
                                humanToScoreTrajectory
                        ),


                        // Score Specimen, delay, Claw Open



                        // PARALLEL Park, Scoring Arm into Bucket Intake Pos, Slide/Arm into Base Position, Intake Full Out
                        new ParallelAction(
                                TrajectoryActionPark
                        )


                        //scoring.scoringArmScore(),

                        //claw.openClaw(),
                        //scoring.LS_ArmBasePos(),
                        //claw.closeClaw(),
                        //scoring.scoringArmScore(),
                        //scoring.LS_ArmBasePos(),
                        //claw.closeClaw(),
                        //scoring.scoringArmScore(),
                        //scoring.LS_ArmBasePos(),
                        //claw.closeClaw(),
                        //scoring.scoringArmScore(),

                         */


                )
        );


    }
}
