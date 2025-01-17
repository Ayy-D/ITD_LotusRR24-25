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

                return false;
            }
        }
        public Action rotationBasePos() {
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
        public Action intakeWheelsOUT() { return new Intake.IntakeWheelsOUT(); }
    }


    @Override
    public void runOpMode() {
        double waitTime = 0.5;

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
        Intake intake = new Intake(hardwareMap);
        LS_Scoring scoring = new LS_Scoring(hardwareMap);
        rotation rotation = new rotation(hardwareMap);



        //Preload Specimen Score
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYConstantHeading(yPose[1])
                .waitSeconds(waitTime);

        //Move to grab floor spec 1
        TrajectoryActionBuilder tab2 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(xPose[2], yPose[2]), angles[2])
                .waitSeconds(waitTime);

        //Move to drop floor spec 1
        TrajectoryActionBuilder tab3 = drive.actionBuilder(pick1Pose)
                .strafeToLinearHeading(new Vector2d(xPose[3], yPose[3]), angles[3])
                .waitSeconds(waitTime/2);

        //Move to grab floor spec 2
        TrajectoryActionBuilder tab4 = drive.actionBuilder(human1Pose)
                .strafeToLinearHeading(new Vector2d(xPose[4], yPose[4]), angles[4])
                .waitSeconds(waitTime);

        //Move to drop floor spec 2
        TrajectoryActionBuilder tab5 = drive.actionBuilder(pick2Pose)
                .strafeToLinearHeading(new Vector2d(xPose[5], yPose[5]), angles[5])
                .waitSeconds(waitTime/2);

        //Move to grab floor spec 3
        TrajectoryActionBuilder tab6 = drive.actionBuilder(human2Pose)
                .strafeToLinearHeading(new Vector2d(xPose[6], yPose[6]), angles[6])
                .waitSeconds(waitTime);

        //Move to drop floor spec 3
        TrajectoryActionBuilder tab7 = drive.actionBuilder(pick3Pose)
                .strafeToLinearHeading(new Vector2d(xPose[7], yPose[7]), angles[7])
                .waitSeconds(waitTime/2);

        //Move to grab spec 1 from wall
        TrajectoryActionBuilder tab8 = drive.actionBuilder(human3Pose)
                .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(90))
                    .waitSeconds(waitTime/5)
                .strafeToLinearHeading(new Vector2d(xPose[8], yPose[8]), angles[8])
                    .waitSeconds(waitTime/2);

        //Move to score specimen 1 from wall
        TrajectoryActionBuilder tab9 = drive.actionBuilder(firstWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[9], yPose[9]), angles[9])
                    .waitSeconds(waitTime);


        //Move to wall grab spec 2
        TrajectoryActionBuilder tab10 = drive.actionBuilder(firstScorePose)
                .strafeToLinearHeading(new Vector2d(-40, 55), Math.toRadians(89.99999))
                    .waitSeconds(waitTime/5)
                .strafeToLinearHeading(new Vector2d(xPose[10], yPose[10]), angles[10])
                    .waitSeconds(waitTime/2);

        //Move to score specimen 2 from wall
        TrajectoryActionBuilder tab11 = drive.actionBuilder(secondWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[11], yPose[11]), angles[11])
                    .waitSeconds(waitTime);

        //Move to wall grab spec 2
        TrajectoryActionBuilder tab12 = drive.actionBuilder(secondScorePose)
                .strafeToLinearHeading(new Vector2d(-40, 55), Math.toRadians(89.99999))
                .waitSeconds(waitTime/5)
                .strafeToLinearHeading(new Vector2d(xPose[12], yPose[12]), angles[12])
                .waitSeconds(waitTime);

        //Move to score specimen 2 from wall
        TrajectoryActionBuilder tab13 = drive.actionBuilder(thirdWallGrabPose)
                .strafeToLinearHeading(new Vector2d(xPose[13], yPose[13]), angles[13])
                .waitSeconds(waitTime);


        //Park
        Action TrajectoryActionPark = drive.actionBuilder(thirdScorePose)
                .strafeToLinearHeading(new Vector2d(-28, 40), Math.toRadians(315))
                .build();



        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(intake.intakeInitPos());
        Actions.runBlocking(rotation.rotationBasePos());

         while (!isStopRequested() && !opModeIsActive()) {
         telemetry.update();
         }

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

                        //          SCORE PRELOAD SPECIMEN
                        // PARALLEL Slide & Arm into Scoring Position, Move Intake to Base Pos, initToScoreTrajectory,
                        new ParallelAction(
                                scoring.LS_SPECScorePos(),
                                intake.intakeBasePos(),
                                initToScoreTrajectory

                        ),



                        //          MOVE AND PREP TO FLOOR/WALL GRABS
                        // Score Preload Specimen, delay, Claw Open
                        scoring.LS_SPECPullPos(),

                        claw.openClaw()

                        /*
                        // PARALLEL Move Preload Score to First Pickup, Move Intake to Halfway, Slide/Arm Down to Intake Position
                        new ParallelAction(
                                intake.intakeHalfwayPos(),
                                scoring.LS_SPECBasePos(),
                                scoreToPick1

                        ),

                        //          FLOOR GRAB #1
                        // PARALLEL Extend Intake #1, Intake Wheels IN, Wait for Intake to Grab
                        new ParallelAction(
                                intake.intakeWheelsIN()
                                ,intake.intakeFullOutPos()
                        ),

                        // PARALLEL Stop Intake Wheels, Move Intake to Halfway, Floor to Human Trajectory #1
                        new ParallelAction(
                                intake.intakeWheelsOFF()
                                ,intake.intakeHalfwayPos()
                                ,pick1ToHuman1

                        ),

                        // PARALLEL Intake wheels OUT, Slight delay, Human to pickup 2
                        new ParallelAction(
                                intake.intakeWheelsOUT()
                                ,human1toPick2
                        ),


                        //          FLOOR GRAB #2
                        // PARALLEL Extend Intake #2, Intake Wheels IN, Wait for Intake to Grab
                        new ParallelAction(
                                intake.intakeWheelsIN()
                                ,intake.intakeFullOutPos()
                        ),

                        // PARALLEL Stop Intake Wheels, Move Intake to Halfway, Floor to Human Trajectory #2
                        new ParallelAction(
                                intake.intakeWheelsOFF()
                                ,intake.intakeHalfwayPos()
                                //pick2ToHuman2
                        ),

                        // PARALLEL Intake wheels OUT, Slight Delay, Human to Pickup 3
                        new ParallelAction(
                                intake.intakeWheelsOUT()
                                //wait 0.5 s
                                ,human2ToPick3
                        ),


                        //          FLOOR GRAB #3
                        // PARALLEL Extend Intake #3, Intake Wheels IN, Wait for Intake to Grab
                        new ParallelAction(
                                intake.intakeWheelsIN()
                                ,intake.intakeFullOutPos()
                                //wait(125)
                        ),

                        // PARALLEL Stop Intake Wheels, Move Intake to Halfway, Floor to Human Trajectory #3
                        new ParallelAction(
                                intake.intakeWheelsOFF()
                                ,intake.intakeHalfwayPos()
                                //pick3ToHuman3
                        ),



                        //          MOVE TO WALL GRAB
                        // PARALLEL Intake Wheels OUT, Slight Delay, Move Intake to Base Pos, Move to Human
                        new ParallelAction(
                                intake.intakeWheelsOUT()
                                //wait(125),
                                ,intake.intakeBasePos()
                                //human3ToWall
                        ),



                        //          WALL SPECIMEN #1
                        // Claw close
                        claw.closeClaw(),

                        // PARALLEL Slide/Arm into Scoring Position, Move Intake to Base Pos, humanToScoreTrajectory #1,
                        new ParallelAction(
                                scoring.LS_SPECScorePos()
                                ,wallToScore1
                        ),


                        // Score Specimen, delay, Claw Open
                        scoring.LS_ArmSPECPullPos(),
                        // 0.125s delay
                        claw.openClaw(),


                        // PARALLEL Slide/Arm into Intake Position, scoreToHumanTrajectory #1
                        new ParallelAction(
                                scoreToHumanTrajectory2
                                ,scoring.LS_SPECBasePos()

                        ),



                        //          WALL SPECIMEN #2
                        // Claw Close
                        claw.closeClaw(),

                        // PARALLEL Slide/Arm into Scoring Position, Move Intake to Base Pos, humanToScoreTrajectory #2
                        new ParallelAction(
                                scoring.LS_SPECScorePos()
                                ,humanToScoreTrajectory2
                        ),


                        // Score Specimen, delay, Claw Open
                        scoring.LS_ArmSPECPullPos(),
                        // 0.125s delay
                        claw.openClaw(),

                        // PARALLEL Slide/Arm into Intake Position, scoreToHumanTrajectory #2
                        new ParallelAction(
                                scoreToHumanTrajectory3
                        ),



                        //          WALL SPECIMEN #3
                        // Claw Close
                        claw.closeClaw(),

                        // PARALLEL Slide/Arm into Scoring Position, Move Intake to Base Pos, humanToScoreTrajectory #3,
                        new ParallelAction(
                                scoring.LS_SPECScorePos()
                                ,humanToScoreTrajectory3
                        ),


                        // Score Specimen, delay, Claw Open
                        scoring.LS_ArmSPECPullPos(),
                        // 0.125s delay
                        claw.openClaw(),

                        // PARALLEL Park, Scoring Arm into Bucket Intake Pos, Slide/Arm into Base Position, Intake Full Out
                        new ParallelAction(
                                TrajectoryActionPark
                        )

                        */

                        //scoring.scoringArmScore(),

                        //claw.openClaw(),
                        //scoring.LS_SPECBasePos(),
                        //claw.closeClaw(),
                        //scoring.scoringArmScore(),
                        //scoring.LS_SPECBasePos(),
                        //claw.closeClaw(),
                        //scoring.scoringArmScore(),
                        //scoring.LS_SPECBasePos(),
                        //claw.closeClaw(),
                        //scoring.scoringArmScore(),




                        //SPECIMEN PATH
                        /*
                        initToScoreTrajectory,

                        scoreToPick1, //preload score to first floor grab
                        pick1ToHuman1, //first floor grab to human
                        human1toPick2, //human to second floor grab
                        pick2ToHuman2, //second floor grab to human
                        human2ToPick3, //human to third floor grab
                        pick3ToHuman3, //third floor grab to human

                        human3ToWall, // human to first wall spec
                        wallToScore1, // wall spec grab to score first wall spec

                        scoreToHumanTrajectory2, // move to second wall spec
                        humanToScoreTrajectory2, // score second wall spec
                        scoreToHumanTrajectory3, // move to third wall spec
                        humanToScoreTrajectory3, // score third wall spec

                         */




                )
        );


    }
}
