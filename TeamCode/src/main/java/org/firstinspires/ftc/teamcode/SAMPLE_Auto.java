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
@Autonomous(name = "SAMPLE Auto", group = "Autonomous")
public class SAMPLE_Auto extends LinearOpMode {
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
                inUD.setPosition(0.85);
                inTwist.setPosition(0.56);

                return false;
            }
        }
        public Action intakeFullOutPos() { return new Intake.IntakeFullOut(); }

        public class IntakeTransfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inArmR.setPosition(0.135);
                inArmL.setPosition(0.135);
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

        Pose2d initialPose = new Pose2d(38, 60, Math.toRadians(90));
		Pose2d prepScore = new Pose2d(52.5, 52.5, Math.toRadians(45));
        Pose2d scoringPose = new Pose2d(54,54, Math.toRadians(45));

		Pose2d prepP1 = new Pose2d(48.25,49, Math.toRadians(90));
        Pose2d pickup1 = new Pose2d(48.25,46, Math.toRadians(90));
		
		Pose2d prepP2 = new Pose2d(58.5, 49, Math.toRadians(90));
        Pose2d pickup2 = new Pose2d(58.5, 46, Math.toRadians(90));

		Pose2d prepP3 = new Pose2d(54, 47, Math.toRadians(135));
        Pose2d pickup3 = new Pose2d(56, 44, Math.toRadians(135));

        Pose2d midPark = new Pose2d(56, 44, Math.toRadians(0));



        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Claw claw = new Claw(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        LS_Scoring scoring = new LS_Scoring(hardwareMap);
        rotation rotation = new rotation(hardwareMap);



        //init to prepScore
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(52.5, 52.5), Math.toRadians(45))
                .waitSeconds(1);

        //prepScore to score preload
        TrajectoryActionBuilder tab2 = drive.actionBuilder(prepScore)
                .strafeToLinearHeading(new Vector2d(54, 54), Math.toRadians(45))
                .waitSeconds(1);

        //score preload to prep pickup 1
        TrajectoryActionBuilder tab3 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(48.25, 49), Math.toRadians(90))
                .waitSeconds(1);

        //prep pickup 1 to pickup 1
        TrajectoryActionBuilder tab4 = drive.actionBuilder(prepP1)
                .strafeToLinearHeading(new Vector2d(48.25, 46), Math.toRadians(90))
                .waitSeconds(1);

        // pickup 1 to prepScore
        TrajectoryActionBuilder tab5 = drive.actionBuilder(pickup1)
                .strafeToLinearHeading(new Vector2d(52.5, 52.5), Math.toRadians(45))
                .waitSeconds(1);

        //prepScore to score 1
        TrajectoryActionBuilder tab6 = drive.actionBuilder(prepScore)
                .strafeToLinearHeading(new Vector2d(54, 54), Math.toRadians(45))
                .waitSeconds(1);


        // score 1 to prep pickup 2
        TrajectoryActionBuilder tab7 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(58.5, 49), Math.toRadians(90))
                .waitSeconds(1);

        // prep pickup 2 to pickup 2
        TrajectoryActionBuilder tab8 = drive.actionBuilder(prepP2)
                .strafeToLinearHeading(new Vector2d(58.5, 46), Math.toRadians(45))
                .waitSeconds(1);

        //pickup 2 to prep score
        TrajectoryActionBuilder tab9 = drive.actionBuilder(pickup2)
                .strafeToLinearHeading(new Vector2d(52.5, 52.5), Math.toRadians(45))
                .waitSeconds(1);

        // prep score to score 2
        TrajectoryActionBuilder tab10 = drive.actionBuilder(prepScore)
                .strafeToLinearHeading(new Vector2d(54, 54), Math.toRadians(45))
                .waitSeconds(1);

        // score 2 to prep pickup 3
        TrajectoryActionBuilder tab11 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(54, 47), Math.toRadians(135))
                .waitSeconds(1);

        // prep pickup 3 to pickup 3
        TrajectoryActionBuilder tab12 = drive.actionBuilder(prepP3)
                .strafeToLinearHeading(new Vector2d(56, 44), Math.toRadians(135))
                .waitSeconds(1);

        //pickup 3 to prep score
        TrajectoryActionBuilder tab13 = drive.actionBuilder(pickup3)
                .strafeToLinearHeading(new Vector2d(52.5, 52.5), Math.toRadians(45))
                .waitSeconds(1);

        //prep score to score 3
        TrajectoryActionBuilder tab14 = drive.actionBuilder(prepScore)
                .strafeToLinearHeading(new Vector2d(54, 54), Math.toRadians(45))
                .waitSeconds(1);

        //score 3 to midPark
        TrajectoryActionBuilder tab15 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(56, 54), Math.toRadians(0))
                .waitSeconds(1);


        // midPark to park in submersible zone
        Action TrajectoryActionCloseOut = drive.actionBuilder(midPark)
                .strafeToLinearHeading(new Vector2d(26, 8), Math.toRadians(0))
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


        Action initToPrepScore;
        Action prepScoreTosScorePreload;
        Action scorePreloadToPrep1;
        Action prep1ToPickup1;
        Action pickup1ToPrepScore;
        Action prepScoreToScore1;
        Action score1ToPrep2;
        Action prep2ToPickup2;
        Action pickup2ToPrepScore;
        Action prepScoreToScore2;
        Action score2ToPrep3;
        Action prep3ToPickup3;
        Action pickup3ToPrepScore;
        Action prepScoreToScore3;

        Action score3ToMidPark;
        Action midParkToPark;


        initToPrepScore = tab1.build();
        prepScoreTosScorePreload = tab2.build();
        scorePreloadToPrep1 = tab3.build();
        prep1ToPickup1 = tab4.build();
        pickup1ToPrepScore = tab5.build();
        prepScoreToScore1 = tab6.build();
        score1ToPrep2 = tab7.build();
        prep2ToPickup2 = tab8.build();
        pickup2ToPrepScore = tab9.build();
        prepScoreToScore2 = tab10.build();
        score2ToPrep3 = tab11.build();
        prep3ToPickup3 = tab12.build();
        pickup3ToPrepScore = tab13.build();
        prepScoreToScore3 = tab14.build();
        score3ToMidPark = tab15.build();

        Actions.runBlocking(
                new SequentialAction(

                        //          SCORE PRELOAD SAMPLE
                        // PARALLEL Slide and Arm into Scoring Position Bucket Pointing UP, Intake to Halfway, initToPrepScore
                        new ParallelAction(
                                initToPrepScore
                                ,rotation.rotationUPPos()
                                ,scoring.LS_SAMPLEScorePos()
                                ,intake.intakeHalfwayPos()
                        ),

                        //Bucket Down
                        new SequentialAction(

                                scoring.LS_BucketTipPos()
                                ,prepScoreTosScorePreload
                        ),



                        //          SCORE FIRST GROUND SAMPLE
                        // PARALLEL scoreInitToPickup1, Slide and Arm into Intake/TELEOP Pos, Intake FULLOUT and IN
                        new ParallelAction(
                                intake.intakeFullOutPos()
                                ,intake.intakeWheelsIN()
                                ,scoring.LS_TeleOpPos()
                                ,scorePreloadToPrep1
                        ),

                        // Bucket to Intake Pos, Intake Transfer Pos, Intake Wheels OFF,
                        new SequentialAction(
                                prep1ToPickup1,
                                scoring.LS_BucketIntakePos()
                                ,intake.intakeWheelsOFF()
                                ,intake.intakeHalfwayPos()

                        ),


                        //Intake Transfer Pos, wait 0.5 for collapse, Intake Wheels OUT
                        new SequentialAction(
                                intake.intakeHalfwayPos()
                                ,intake.intakeTransferPos()
                                ,intake.intakeWheelsOUT()

                        ),

                        // PARALLEL pickup1ToScore1, Slide and Arm into Scoring Position Bucket Pointing UP, Intake to Halfway
                        new ParallelAction(
                                scoring.LS_SAMPLEScorePos()
                                ,intake.intakeHalfwayPos()
                                ,pickup1ToPrepScore
                        ),

                        // Bucket Down
                        new SequentialAction(
                                scoring.LS_BucketTipPos()
                                ,prepScoreToScore1

                        ),



                        //          SCORE SECOND GROUND SAMPLE
                        // PARALLEL score1ToPickup2, Slide and Arm into Intake/TELEOP Pos, Intake FULLOUT and IN
                        new ParallelAction(
                                intake.intakeFullOutPos()
                                ,intake.intakeWheelsIN()
                                ,scoring.LS_TeleOpPos()
                                ,score1ToPrep2
                        ),

                        // Bucket to Intake Pos, Intake Transfer Pos, Intake Wheels OFF,
                        new SequentialAction(
                                prep2ToPickup2,
                                scoring.LS_BucketIntakePos()
                                ,intake.intakeWheelsOFF()
                                ,intake.intakeHalfwayPos()

                        ),


                        //Intake Transfer Pos, wait 0.5 for collapse, Intake Wheels OUT
                        new SequentialAction(
                                intake.intakeHalfwayPos()
                                ,intake.intakeTransferPos()
                                ,intake.intakeWheelsOUT()

                        ),

                        // PARALLEL pickup1ToScore1, Slide and Arm into Scoring Position Bucket Pointing UP, Intake to Halfway
                        new ParallelAction(
                                scoring.LS_SAMPLEScorePos()
                                ,intake.intakeHalfwayPos()
                                ,pickup2ToPrepScore
                        ),

                        // Bucket Down
                        new SequentialAction(
                                prepScoreToScore2
                                ,scoring.LS_BucketTipPos()
                        ),



                        //          SCORE THIRD GROUND SAMPLE
                        // PARALLEL score2ToPickup3, Slide and Arm into Intake/TELEOP Pos, Intake FULLOUT and IN
                        new ParallelAction(
                                intake.intakeFullOutPos()
                                ,intake.intakeWheelsIN()
                                ,scoring.LS_TeleOpPos()
                                ,score2ToPrep3
                        ),

                        // Bucket to Intake Pos, Intake Transfer Pos, Intake Wheels OFF,
                        new SequentialAction(
                                prep3ToPickup3
                                ,scoring.LS_BucketIntakePos()
                                ,intake.intakeWheelsOFF()
                                ,intake.intakeHalfwayPos()

                        ),


                        //Intake Transfer Pos, wait 0.5 for collapse, Intake Wheels OUT
                        new SequentialAction(
                                intake.intakeHalfwayPos()
                                ,intake.intakeTransferPos()
                                ,intake.intakeWheelsOUT()

                        ),

                        // PARALLEL pickup1ToScore1, Slide and Arm into Scoring Position Bucket Pointing UP, Intake to Halfway
                        new ParallelAction(
                                scoring.LS_SAMPLEScorePos()
                                ,intake.intakeHalfwayPos()
                                ,pickup3ToPrepScore
                        ),

                        // Bucket Down
                        new SequentialAction(
                                prepScoreToScore3
                                ,scoring.LS_BucketTipPos()
                        ),

                        new ParallelAction(
                                intake.intakeInitPos()
                                ,scoring.LS_TeleOpPos()
                                ,score3ToMidPark

                        ),

                        TrajectoryActionCloseOut

                )
        );


    }
}
