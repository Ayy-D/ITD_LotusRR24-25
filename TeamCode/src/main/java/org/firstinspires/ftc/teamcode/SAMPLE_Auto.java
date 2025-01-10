package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

        public class LS_ArmBase implements Action {
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
                rotL.setTargetPosition(195);
                rotR.setTargetPosition(195);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }
        public  Action LS_ArmBasePos() {
            return new LS_Scoring.LS_ArmBase();
        }

        public class LS_ArmSPECScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                scR.setPosition(0.48);
                scL.setPosition(0.48);
                scUD.setPosition(0.4);

                rotL.setPower(0.4);
                rotL.setTargetPosition(195);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setPower(0.4);
                rotR.setTargetPosition(195);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                sL.setPower(0.9);
                sL.setTargetPosition(1835);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setPower(0.9);
                sR.setTargetPosition(1820);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }
        public Action LS_ArmSPECScorePos() {
            return new LS_Scoring.LS_ArmSPECScore();
        }

        public class LS_ArmScorePull implements Action {
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

                rotL.setPower(0.4);
                rotR.setPower(0.4);
                rotL.setTargetPosition(200);
                rotR.setTargetPosition(200);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }
        public Action LS_ArmScorePullPos() {
            return new LS_Scoring.LS_ArmScorePull();
        }

        public class LS_ArmSAMPLEScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                scR.setPosition(0.48);
                scL.setPosition(0.48);
                scUD.setPosition(0.4);

                rotL.setPower(0.4);
                rotL.setTargetPosition(195);
                rotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotR.setPower(0.4);
                rotR.setTargetPosition(195);
                rotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                sL.setPower(0.9);
                sL.setTargetPosition(1835);
                sL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sR.setPower(0.9);
                sR.setTargetPosition(1820);
                sR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                return false;
            }
        }
        public Action LS_ArmSAMPLEScorePos() {
            return new LS_Scoring.LS_ArmSAMPLEScore();
        }

        public class LS_ARM_BucketTip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                scUD.setPosition(0.7);
                return false;
            }
        }
        public Action LS_ARM_BucketTipPos() { return new LS_Scoring.LS_ARM_BucketTip(); }

        public class LS_Arm_TeleOp implements Action {
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
        public Action LS_Arm_TeleOpPos() { return new LS_Scoring.LS_Arm_TeleOp(); }
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
        public Action intakeInitPos() {
            return new IntakeHold.IntakeInit();
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
        public Action intakeBasePos() { return new IntakeHold.IntakeBase(); }

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
        public Action intakeHalfwayPos() { return new IntakeHold.IntakeHalfway(); }

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
        public Action intakeFullOutPos() { return new IntakeHold.IntakeFullOut(); }

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
        public Action intakeTransferPos() { return new IntakeHold.IntakeTransfer(); }



        public class IntakeWheelsIN implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(-1);
                inL.setPower(1);;
                return false;
            }
        }
        public Action intakeWheelsIN() { return new IntakeHold.IntakeWheelsIN(); }


        public class IntakeWheelsOFF implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(0);
                inL.setPower(0);
                return false;
            }
        }
        public Action intakeWheelsOFF() { return new IntakeHold.IntakeWheelsOFF(); }


        public class IntakeWheelsOUT implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                inR.setPower(1);
                inL.setPower(-1);
                return false;
            }
        }
        public Action intakeWheelsOUT() { return new IntakeHold.IntakeWheelsOUT(); }
    }



    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(38, 60, Math.toRadians(90));
        Pose2d scoringPose = new Pose2d(54,54, Math.toRadians(45));
        Pose2d pickup1 = new Pose2d(48,48, Math.toRadians(90));
        Pose2d pickup2 = new Pose2d(57,48, Math.toRadians(90));
        Pose2d pickup3 = new Pose2d(57,46, Math.toRadians(125));
        //Pose2d intakePose = new Pose2d(-48, 46, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Claw claw = new Claw(hardwareMap);
        IntakeHold intake = new IntakeHold(hardwareMap);
        LS_Scoring scoring = new LS_Scoring(hardwareMap);



        //init to score
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(54, 54), Math.toRadians(45))
                .waitSeconds(0.5);

        // Build the trajectory
        //score to pickup 1
        TrajectoryActionBuilder tab2 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(47.5, 48), Math.toRadians(90))
                .waitSeconds(0.5);

        // pickup 1 to score
        TrajectoryActionBuilder tab3 = drive.actionBuilder(pickup1)
                .strafeToLinearHeading(new Vector2d(54, 54), Math.toRadians(45))
                .waitSeconds(0.5);

        // score to pickup 2
        TrajectoryActionBuilder tab4 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(57.5, 48), Math.toRadians(90))
                .waitSeconds(0.5);

        //pickup 2 to score
        TrajectoryActionBuilder tab5 = drive.actionBuilder(pickup2)
                .strafeToLinearHeading(new Vector2d(54, 54), Math.toRadians(45))
                .waitSeconds(0.5);

        //score to pickup 3
        TrajectoryActionBuilder tab6 = drive.actionBuilder(scoringPose)
                .strafeToLinearHeading(new Vector2d(57, 46), Math.toRadians(125))
                .waitSeconds(0.5);

        //pickup 3 to score
        TrajectoryActionBuilder tab7 = drive.actionBuilder(pickup3)
                .strafeToLinearHeading(new Vector2d(54, 54), Math.toRadians(45))
                .waitSeconds(0.5);

        //park in observation zone
        Action TrajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-34.5, 57))
                .build();



        //Actions.runBlocking(claw.closeClaw());
        //Actions.runBlocking(intakeHold.intakeHoldBase());
        /***
         while (!isStopRequested() && !opModeIsActive()) {
         telemetry.update();
         }
         ***/
        // Wait for the start signal
        waitForStart();
        if (isStopRequested()) return;


        Action initToScoreTrajectory;
        Action scoreToPickup1;
        Action pickup1ToScore;
        Action scoreToPickup2;
        Action pickup2ToScore;
        Action scoreToPickup3;
        Action pickup3ToScore;

        initToScoreTrajectory = tab1.build();
        scoreToPickup1 = tab2.build();
        pickup1ToScore = tab3.build();
        scoreToPickup2 = tab4.build();
        pickup2ToScore = tab5.build();
        scoreToPickup3 = tab6.build();
        pickup3ToScore = tab7.build();

        Actions.runBlocking(
                new SequentialAction(
                        //          SCORE PRELOAD SAMPLE
                        // PARALLEL Slide and Arm into Scoring Position Bucket Pointing UP, Intake to Halfway, initToScoreTrajectory
                        // Bucket Down

                        //          SCORE FIRST GROUND SAMPLE
                        // PARALLEL scoreToPickup1, Slide and Arm into Intake Pos, Intake FULLOUT and IN
                        // Wait 0.5 for pickup
                        // Intake Transfer Pos, wait 0.5 for collapse, Intake Wheels OUT,
                        // PARALLEL pickup1ToScore, Slide and Arm into Scoring Position Bucket Pointing UP, Intake to Halfway
                        // Bucket Down

                        //          SCORE SECOND GROUND SAMPLE
                        // PARALLEL scoreToPickup2, Slide and Arm into Intake Pos, Intake FULLOUT and IN
                        // Wait 0.5 for pickup
                        // Intake Transfer Pos, wait 0.5 for collapse, Intake Wheels OUT
                        // PARALLEL pickup2ToScore, Slide and Arm into Scoring Position Bucket Pointing UP, Intake to Halfway
                        // Bucket Down

                        //          SCORE THIRD GROUND SAMPLE
                        // PARALLEL scoreToPickup3, Slide and Arm into Intake Pos, Intake FULLOUT and IN
                        // Wait 0.5 for pickup
                        // Intake Transfer Pos, wait 0.5 for collapse, Intake Wheels OUT
                        // PARALLEL pickup3ToScore, Slide and Arm into Scoring Position Bucket Pointing UP, Intake to Base Pos
                        // Bucket Down

                        //RESET FOR TELEOP AND PARK

                        // PARALLEL
                        initToScoreTrajectory,
                        //slide score with bucket
                        scoreToPickup1,
                        //pickup sample
                        pickup1ToScore,
                        //slide score with bucket
                        scoreToPickup2,
                        //pickup sample
                        pickup2ToScore,
                        //slide score with bucket
                        scoreToPickup3,
                        //pickup sample
                        pickup3ToScore,
                        //slide score with bucket
                        TrajectoryActionCloseOut



                )
        );


    }
}
