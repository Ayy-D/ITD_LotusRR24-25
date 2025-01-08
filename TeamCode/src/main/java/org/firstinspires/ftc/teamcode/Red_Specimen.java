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
@Autonomous(name = "Red Specimen", group = "Autonomous")
public class Red_Specimen extends LinearOpMode {
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

        Pose2d initialPose = new Pose2d(0, 60, Math.toRadians(270));
        Pose2d scoringPose = new Pose2d(0,34.5, Math.toRadians(270));
        Pose2d scoreToPick1Pose = new Pose2d(-34, 42, Math.toRadians(45));
        Pose2d pick1ToHuman1Pose = new Pose2d(-36, 48, Math.toRadians(315));
        Pose2d human1toPick2Pose = new Pose2d(-44, 42, Math.toRadians(45));
        Pose2d pick2ToHuman2Pose = new Pose2d(-48, 48, Math.toRadians(45));
        Pose2d human2ToPick3Pose = new Pose2d(-54, 42, Math.toRadians(315));
        Pose2d pick3ToHuman3Pose = new Pose2d(-54, 48, Math.toRadians(315));
        Pose2d human3ToWallPose = new Pose2d(-40, 58, Math.toRadians(90));

        Pose2d intakePose = new Pose2d(-48, 58, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Claw claw = new Claw(hardwareMap);
        IntakeHold intake = new IntakeHold(hardwareMap);
        LS_Scoring scoring = new LS_Scoring(hardwareMap);



        //Preload Specimen Score
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYConstantHeading(34.5)
                .waitSeconds(0.5);

        //Move to grab floor spec 1
        TrajectoryActionBuilder tab2 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(-34, 42), Math.toRadians(45))
                .waitSeconds(0.5);

        //Move to drop floor spec 1
        TrajectoryActionBuilder tab3 = drive.actionBuilder(scoreToPick1Pose)
                .strafeToLinearHeading(new Vector2d(-36, 48), Math.toRadians(315))
                .waitSeconds(0.5);

        //Move to grab floor spec 2
        TrajectoryActionBuilder tab4 = drive.actionBuilder(pick1ToHuman1Pose)
                .strafeToLinearHeading(new Vector2d(-44, 42), Math.toRadians(45))
                .waitSeconds(0.5);

        //Move to drop floor spec 2
        TrajectoryActionBuilder tab5 = drive.actionBuilder(human1toPick2Pose)
                .strafeToLinearHeading(new Vector2d(-48, 48), Math.toRadians(315))
                .waitSeconds(0.5);

        //Move to grab floor spec 3
        TrajectoryActionBuilder tab6 = drive.actionBuilder(pick2ToHuman2Pose)
                .strafeToLinearHeading(new Vector2d(-54, 42), Math.toRadians(45))
                .waitSeconds(0.5);

        //Move to drop floor spec 3
        TrajectoryActionBuilder tab7 = drive.actionBuilder(human2ToPick3Pose)
                .strafeToLinearHeading(new Vector2d(-54, 48), Math.toRadians(315))
                .waitSeconds(0.5);

        //Move to grab spec 1 from wall
        TrajectoryActionBuilder tab8 = drive.actionBuilder(pick3ToHuman3Pose)
                .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-40, 58), Math.toRadians(90))
                .waitSeconds(0.5);

        //Move to score specimen 1 from wall
        TrajectoryActionBuilder tab9 = drive.actionBuilder(human3ToWallPose)
                .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-40, 58), Math.toRadians(90))
                .waitSeconds(0.5);


        //Move to wall grab spec 2/3
        TrajectoryActionBuilder tab10 = drive.actionBuilder(intakePose)
                .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(89.999))
                .strafeToLinearHeading(new Vector2d(-40, 58), Math.toRadians(90))
                .waitSeconds(0.5);

        //Move to score specimen 2/3 from wall
        TrajectoryActionBuilder tab11 = drive.actionBuilder(scoringPose)
                .strafeToSplineHeading(new Vector2d(0, 34), Math.toRadians(-90))
                .waitSeconds(0.5);


        //Park
        Action TrajectoryActionPark = tab1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-24, 45), Math.toRadians(315))
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

        Action scoreToHumanTrajectory = tab10.build();
        Action humanToScoreTrajectory = tab11.build();


        Actions.runBlocking(
                new SequentialAction(
                        initToScoreTrajectory,

                        scoreToPick1,
                        pick1ToHuman1,
                        human1toPick2,
                        pick2ToHuman2,
                        human2ToPick3,
                        pick3ToHuman3,

                        human3ToWall, // move to first wall spec
                        wallToScore1, // score first wall spec

                        scoreToHumanTrajectory, // move to second wall spec
                        humanToScoreTrajectory, // score second wall spec
                        scoreToHumanTrajectory, // move to third wall spec
                        humanToScoreTrajectory, // score third wall spec
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