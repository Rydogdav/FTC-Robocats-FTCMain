/**
 * Created by rydo on 9/17/2017.
 */
package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import static android.R.transition.move;


public abstract class FTCMain {
    //Movement motors
    public static DcMotor motorFLeft = null;
    public static DcMotor motorFRight = null;
    public static DcMotor motorBLeft = null;
    public static DcMotor motorBRight = null;

    //Movement motor variables
    public static double motorFLeftv;
    public static double motorFRightv;
    public static double motorBLeftv;
    public static double motorBRightv;
    
    //Movement motor variables flip based on Fwd/Bwd mode (includes @param motorOffset.
    public static double motorFLeftFwd;
    public static double motorFRightFwd;
    public static double motorBLeftFwd;
    public static double motorBRightFwd;

    //Misc. motors
    public static DcMotor motorLShoot = null;
    public static DcMotor motorRShoot = null;

    //Servos
    public static Servo servoShooterPipe = null;
    public static Servo servoShooterGate = null;

    //Sensors
    public static GyroSensor gyroMain = null;
    public static OpticalDistanceSensor distanceMainF = null;

    public static double motorOffset = 0.2;
    public static short maxMotorPower = 1;
    public static double motorFLeftAdj = 0.95;
    public static double motorFRightAdj = 1;
    public static double motorBRightAdj = 0.95;
    public static double motorBLeftAdj = 1.3;

    public abstract class Init {

        public void initBaseRobit(HardwareMap ahwMap) {
            HardwareMap hwMap = null;
            hwMap = ahwMap; //Crossing over the hardwareMap class from a opmode class to a non-opmode class.

            //Movement motor hwMaps
            motorFLeft = hwMap.get(DcMotor.class, "motorFLeft");
            motorFRight = hwMap.get(DcMotor.class, "motorFRight");
            motorBLeft = hwMap.get(DcMotor.class, "motorBLeft");
            motorBRight = hwMap.get(DcMotor.class, "motorBRight");

            //Misc. motor hwMaps
            motorLShoot = hwMap.get(DcMotor.class, "motorLShoot");
            motorRShoot = hwMap.get(DcMotor.class, "motorRShoot");

            //Servos hwMaps
            servoShooterPipe = hwMap.get(Servo.class, "servoShooterPipe");
            servoShooterGate = hwMap.get(Servo.class, "servoShooterGate");

            //Sensor hwMaps
            gyroMain = hwMap.get(GyroSensor.class, "gyroMain");
            distanceMainF = hwMap.get(OpticalDistanceSensor.class, "distanceMainF");

            //Start status setting


        }
    }

    public static void motorAdjustments(){ //Adjusts the motors so that they do not overlap
        if (motorFLeftv != 0) {
            motorFLeftv *= motorFLeftFwd * motorFLeftAdj;
        }
        if (motorFRightv != 0) {
            motorFRightv *= motorFRightFwd * motorFRightAdj;
        }
        if (motorBLeftv != 0) {
            motorBLeftv *= motorBLeftFwd * motorBLeftAdj;
        }
        if (motorBRightv != 0) {
            motorBRightv *= motorBRightFwd * motorBRightAdj;
        }
        /*
        if (motorFLeftv != 0) {
            motorFLeftv *= motorFLeftAdj;
        }
        if (motorBRightv != 0) {
            motorBRightv *= motorBRightAdj;
        }
        if (motorFRightv != 0) {
            motorFRightv *= motorFRightAdj;
        }
        if (motorBLeftv != 0) {
            motorBLeftv *= motorBLeftAdj;
        }
        */
        if (motorFLeft.getDirection() == DcMotorSimple.Direction.REVERSE) { //Stops the drifting of the robot
            motorFLeftFwd = 1 - motorOffset;
        }
        if (motorFRight.getDirection() == DcMotorSimple.Direction.REVERSE) {
            motorFRightFwd = 1 - motorOffset;
        }
        if (motorBLeft.getDirection() == DcMotorSimple.Direction.REVERSE) {
            motorBLeftFwd = 1 - motorOffset;
        }
        if (motorBRight.getDirection() == DcMotorSimple.Direction.REVERSE) {
            motorBRightFwd = 1 - motorOffset;
        }
        if (motorFLeft.getDirection() == DcMotorSimple.Direction.FORWARD) {
            motorFLeftFwd = 1 + motorOffset;
        }
        if (motorFRight.getDirection() == DcMotorSimple.Direction.FORWARD) {
            motorFRightFwd = 1 + motorOffset;
        }
        if (motorBLeft.getDirection() == DcMotorSimple.Direction.FORWARD) {
            motorBLeftFwd = 1 + motorOffset;
        }
        if (motorBRight.getDirection() == DcMotorSimple.Direction.FORWARD) {
            motorBRightFwd = 1 + motorOffset;
        }
    }
    public abstract static class DriveTeleOp extends LinearOpMode {

        public static void FieldCentricMecanum(double North, double East, double TurnCW, byte currentGear) {
            double Kf = 1; //Adjustments for forwards power
            double Ks = 1; //Adjustments for strafe power
            double Kt = 1; //Adjustments for turning power

            boolean firstRun = false;

            if (!firstRun){
                motorFLeftv = 0; //Clear the global variables on the first run, just in case.
                motorFRightv = 0;
                motorBRightv = 0;
                motorBLeftv = 0;
            }
            //Convert from field-centric inputs to robot-centric commands
            short currentHeading = (short) gyroMain.getHeading();
            double Forward = +North * Math.cos(currentHeading) + East * Math.sin(currentHeading);
            double Strafe = -North * Math.sin(currentHeading) + East * Math.cos(currentHeading);

            //Scaling outputs for gear input and tuning constants
            Forward = currentGear * Kf * Forward;
            Strafe = currentGear * Ks * Strafe;
            TurnCW = currentGear * Kt * TurnCW;

            //apply inverse kinematics to the scaled input
            motorFLeftv = +Forward + TurnCW + Strafe;
            motorFRightv = +Forward - TurnCW - Strafe;
            motorBLeftv = +Forward + TurnCW - Strafe;
            motorBRightv = +Forward - TurnCW + Strafe;

            motorAdjustments();

            //find maximum input
            double maxInput = Math.abs(motorFLeftv);
            if (Math.abs(motorFRightv) > maxInput) maxInput = Math.abs(motorFRightv);
            if (Math.abs(motorBLeftv) > maxInput) maxInput = Math.abs(motorBLeftv);
            if (Math.abs(motorBRightv) > maxInput) maxInput = Math.abs(motorBRightv);


            //normalize to maximum allowed motor power
            if (maxInput > maxMotorPower) {
                motorFLeftv = maxMotorPower * motorFLeftv / maxInput;
                motorFRightv = maxMotorPower * motorFRightv / maxInput;
                motorBLeftv = maxMotorPower * motorBLeftv / maxInput;
                motorBRightv = maxMotorPower * motorBRightv / maxInput;
            }
            firstRun = true;
        }
    }

    public abstract static class DriveAuton extends LinearOpMode {

        final static int WHEEL_DIAMETER = 4;     //Diameter of the wheel in inches
        final static double WHEEL_DIAMETER_MM = WHEEL_DIAMETER * (25.4);
        final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_MM;
        final static int ENCODER_CPR = 1120;     //Encoder Counts per Revolution
        final static double GEAR_RATIO = 1;      //Gear Ratio

        final static double ROBOT_TURN_CIRCLE_RADIUS = 7.625;
        final static double ROBOT_TURN_CURCUMFERECE = ROBOT_TURN_CIRCLE_RADIUS * Math.PI * 25.4;

        public static void ResetEncoder() { //Resets all motor encoders

            motorFLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motorFLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorFLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public static void ResetEncoderlf() { //Runs Line Follower without encoders

            motorFLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public static void AllStop() { //Stops all the motors
            motorFLeft.setPower(0);
            motorBRight.setPower(0);
            motorBLeft.setPower(0);
            motorFRight.setPower(0);
        }
        public static void HeadingTurn(double HEADING, boolean gearInversion) { //Method for turning
            boolean move = true;
            int gearInversionInt = 1;
            double speedMultiplier = 1;
            double smFinder = 0;
            if (gearInversion) gearInversionInt = -1;
            if (HEADING > 180) {
                HEADING -= 180;
            } else {
                HEADING += 180;
            }
            do {
                Thetacurr = gyroMain.getHeading();
                if (Thetacurr > 180) {
                    Thetacurr -= 180;
                } else {
                    Thetacurr += 180;
                }
                if (Thetacurr > HEADING) {
                    motorFLeft.setPower(-0.3 * gearInversionInt * speedMultiplier); //Runs to position at this power
                    motorBLeft.setPower(-0.3 * gearInversionInt * speedMultiplier);
                    motorFRight.setPower(0.3 * gearInversionInt * speedMultiplier);
                    motorBRight.setPower(0.3 * gearInversionInt * speedMultiplier);
                } else {
                    if (Thetacurr < HEADING) {
                        motorFLeft.setPower(0.3 * gearInversionInt * speedMultiplier); //Runs to position at this power
                        motorBLeft.setPower(0.3 * gearInversionInt * speedMultiplier);
                        motorFRight.setPower(-0.3 * gearInversionInt * speedMultiplier);
                        motorBRight.setPower(-0.3 * gearInversionInt * speedMultiplier);
                    } else {
                        AllStop();
                        ResetEncoder();
                        move = !move;
                    }
                }
            } while (move);
        }


        public static void LinearMove(double DISTANCE, boolean FORWARDS, boolean gearInversion) {
            final double ROTATIONS = DISTANCE / CIRCUMFERENCE;
            final double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
            int DIRECTION_MULTIPLIER;

            ResetEncoder();

            if (FORWARDS == true) {
                DIRECTION_MULTIPLIER = 1;
            } else {
                DIRECTION_MULTIPLIER = -1;
            }
            if (gearInversion) DIRECTION_MULTIPLIER *= -1;
            motorFLeft.setTargetPosition((int) COUNTS); //Sets position in counts
            motorFRight.setTargetPosition((int) COUNTS);
            motorBLeft.setTargetPosition((int) COUNTS);
            motorBRight.setTargetPosition((int) COUNTS);

            motorFLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to position
            motorFRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFLeft.setPower(DIRECTION_MULTIPLIER * 0.5);
            motorFRight.setPower(DIRECTION_MULTIPLIER * 0.5);
            motorBLeft.setPower(DIRECTION_MULTIPLIER * 0.5);
            motorBRight.setPower(DIRECTION_MULTIPLIER * 0.5);
        }

        public static double HEADING_TARGET;
        public static double GYRO_HEADING_NEW;
        public static boolean lfMove = true;


        public static void ASSMoveLF(boolean FORWARDS, boolean gearInversion){
            double HEADING_DELTA;
            final double PerfectColorValue = 0.0825;
            Thetacurr = gyroMain.getHeading();

            if (Thetacurr > 180) {
                HEADING_TARGET = Thetacurr - 180;
            } else {
                HEADING_TARGET = Thetacurr + 180;
            }
            int DIRECTION_MULTIPLIER;
            motorFLeftv = 0.1;
            motorFRightv = 0.1;
            motorBLeftv = 0.1;
            motorBRightv = 0.1;
            if (FORWARDS) {
                DIRECTION_MULTIPLIER = 1;
            } else {
                DIRECTION_MULTIPLIER = -1;
            }
            if (gearInversion) DIRECTION_MULTIPLIER *= -1;
            do {
                Thetacurr = gyroMain.getHeading();
                if (Thetacurr > 180) {
                    GYRO_HEADING_NEW = Thetacurr - 180;
                } else {
                    GYRO_HEADING_NEW = Thetacurr + 180;
                }
                if (GYRO_HEADING_NEW < HEADING_TARGET) {//If Statement logic is inverted due to the headings being set to the 90-270 side instead of the 270-90 side.
                    HEADING_DELTA = (HEADING_TARGET - GYRO_HEADING_NEW) / 12;
                    motorFLeftv =+ HEADING_DELTA;
                    motorBLeftv =+ HEADING_DELTA;
                }
                if (GYRO_HEADING_NEW > HEADING_TARGET) {
                    HEADING_DELTA = (GYRO_HEADING_NEW - HEADING_TARGET) / 12;
                    motorFRightv =+ HEADING_DELTA;
                    motorBRightv =+ HEADING_DELTA;
                }
                if (Thetacurr == HEADING_TARGET) {
                    motorFLeftv = 0.1;
                    motorFRightv = 0.1;
                    motorBLeftv = 0.1;
                    motorBRightv = 0.1;
                }
                if(!lfMove){
                    motorFLeftv = 0;
                    motorBLeftv = 0;
                    motorFRightv = 0;
                    motorBRightv = 0;
                }
                motorFLeft.setPower(DIRECTION_MULTIPLIER * motorFLeftv);
                motorFRight.setPower(DIRECTION_MULTIPLIER * motorFRightv);
                motorBLeft.setPower(DIRECTION_MULTIPLIER * motorBLeftv);
                motorBRight.setPower(DIRECTION_MULTIPLIER * motorBRightv);
                if (distanceMainF.getLightDetected() >= PerfectColorValue)  {
                    lfMove = false;
                }
            } while (lfMove);
            AllStop();
            ResetEncoderlf();
        }
        public static void ASSMove(double DISTANCE, boolean FORWARDS, boolean gearInversion) { //"AutomatedStabilitySystemMove"
            double HEADING_DELTA;
            Thetacurr = gyroMain.getHeading();

            if (Thetacurr > 180) {
                HEADING_TARGET = Thetacurr - 180;
            } else {
                HEADING_TARGET = Thetacurr + 180;
            }
            int DIRECTION_MULTIPLIER;
            motorFLeftv = 0.4;
            motorFRightv = 0.4;
            motorBLeftv = 0.4;
            motorBRightv = 0.4;
            final double ROTATIONS = DISTANCE / CIRCUMFERENCE;
            final double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
            if (FORWARDS) {
                DIRECTION_MULTIPLIER = 1;
            } else {
                DIRECTION_MULTIPLIER = -1;
            }
            if (gearInversion) DIRECTION_MULTIPLIER *= -1;
            boolean move = true;
            motorFLeft.setTargetPosition((int) COUNTS); //Sets position in counts
            motorFRight.setTargetPosition((int) COUNTS);
            motorBLeft.setTargetPosition((int) COUNTS);
            motorBRight.setTargetPosition((int) COUNTS);

            motorFLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to position
            motorFRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            do {
                if (motorFLeft.getCurrentPosition() >= COUNTS ||
                        motorBLeft.getCurrentPosition() >= COUNTS ||
                        motorFRight.getCurrentPosition() >= COUNTS ||
                        motorBRight.getCurrentPosition() >= COUNTS) {
                    move = false;
                    AllStop();
                }
                Thetacurr = gyroMain.getHeading();
                if (Thetacurr > 180) {
                    GYRO_HEADING_NEW = Thetacurr - 180;
                } else {
                    GYRO_HEADING_NEW = Thetacurr + 180;
                }
                if (GYRO_HEADING_NEW < HEADING_TARGET) {//If Statement logic is inverted due to the headings being set to the 90-270 side instead of the 270-90 side.
                    HEADING_DELTA = (HEADING_TARGET - GYRO_HEADING_NEW) / 10;
                    motorFLeftv =+ HEADING_DELTA;
                    motorBLeftv =+ HEADING_DELTA;
                }
                if (GYRO_HEADING_NEW > HEADING_TARGET) {
                    HEADING_DELTA = (GYRO_HEADING_NEW - HEADING_TARGET) / 10;
                    motorFRightv =+ HEADING_DELTA;
                    motorBRightv =+ HEADING_DELTA;
                }
                if (Thetacurr == HEADING_TARGET) {
                    motorFLeftv = 0.4;
                    motorFRightv = 0.4;
                    motorBLeftv = 0.4;
                    motorBRightv = 0.4;
                }
                if(!move){
                    motorFLeftv = 0;
                    motorBLeftv = 0;
                    motorFRightv = 0;
                    motorBRightv = 0;
                }
                motorFLeft.setPower(DIRECTION_MULTIPLIER * motorFLeftv);
                motorFRight.setPower(DIRECTION_MULTIPLIER * motorFRightv);
                motorBLeft.setPower(DIRECTION_MULTIPLIER * motorBLeftv);
                motorBRight.setPower(DIRECTION_MULTIPLIER * motorBRightv);
            } while (move);
            AllStop();
            ResetEncoderlf();
        }

        public static int Xcurr;
        public static int Ycurr;
        public static int Thetacurr;

        public static void Fwd(int Xnew, int Ynew, int Headingfinal, boolean Forwards, boolean gearInversion) {
            double Distance = 0;
            double Thetadelta = 0;
            int Xdelta = Xnew - Xcurr;
            int Ydelta = Ynew - Ycurr;
            if (Xdelta != 0 && Ydelta != 0) {
                Distance = Math.hypot(Xdelta, Ydelta);
                Thetadelta = Math.atan(Ydelta / Xdelta);
                Thetadelta = Math.toDegrees(Thetadelta);
                if (Thetadelta > 0){
                    if (Xnew > 0){
                        Thetadelta = Math.abs(Thetadelta);
                    } else {
                        Thetadelta = Thetadelta + 360;
                    }
                }
                Thetacurr = gyroMain.getHeading();
                if (Thetacurr > Thetadelta) {
                    HeadingTurn(Thetacurr - Thetadelta, gearInversion);
                } else {
                    HeadingTurn(Thetadelta - Thetacurr, gearInversion);
                }

                ASSMove(Distance, Forwards, gearInversion);

                HeadingTurn(Headingfinal, gearInversion);

            } else if (Xdelta == 0) {
                if (Ydelta < 0) {
                    HeadingTurn(0, gearInversion);
                    ;
                    ASSMove(Distance, Forwards, gearInversion);

                    HeadingTurn(Headingfinal, gearInversion);

                } else {
                    HeadingTurn(180, gearInversion);
                    ASSMove(Distance, Forwards, gearInversion);

                    HeadingTurn(Headingfinal, gearInversion);

                }
            }
            else if (Ydelta == 0) {
                if (Xdelta < 0) {
                    HeadingTurn(90, gearInversion);

                    ASSMove(Distance, Forwards, gearInversion);

                    HeadingTurn(Headingfinal, gearInversion);

                } else {
                    HeadingTurn(270, gearInversion);

                    ASSMove(Distance, Forwards, gearInversion);

                    HeadingTurn(Headingfinal, gearInversion);

                }
                Xcurr = Xnew;
                Ycurr = Ynew;
            }
        }
    }
    public abstract static class Shooting extends LinearOpMode {
        public static void ParticleShootTele(short shootpipeMin, short shootpipeMax) {
            servoShooterPipe.setPosition(shootpipeMin);
            SystemClock.sleep(1000);
            servoShooterPipe.setPosition(shootpipeMax);
        }

        public static void ParticleShootAuton() {
            double lshootPower = 0.47;
            double rshootPower = 0.53;
            double shootpipeMin = 0.27;
            double shootpipeMax = 0.53;
            double shootgateMax = 0.78;
            double shootgateMin = 0.47;
            if ((motorLShoot.getPower() != lshootPower) || (motorRShoot.getPower() != rshootPower)) {
                motorLShoot.setPower(lshootPower);
                motorRShoot.setPower(rshootPower);
                SystemClock.sleep(3000);
            }
            servoShooterPipe.setPosition(shootpipeMin);
            SystemClock.sleep(1000);
            servoShooterPipe.setPosition(shootpipeMax);
            SystemClock.sleep(1000);
        }
        public static void ParticleShootAuton2()
        {
            double shootgateMax = 0.78;
            double shootgateMin = 0.47;
            motorLShoot.setPower(0);
            motorRShoot.setPower(0);
            servoShooterGate.setPosition(shootgateMax);
            SystemClock.sleep(1000);
            servoShooterGate.setPosition(shootgateMin);
            ParticleShootAuton();
            motorLShoot.setPower(0);
            motorRShoot.setPower(0);
        }

        public static void ParticleShootAuton2_old() {
            double lshootPower = 0.47;
            double rshootPower = 0.53;
            double shootpipeMin = 0.27;
            double shootpipeMax = 0.53;
            double shootgateMax = 0.78;
            double shootgateMin = 0.47;
            if (motorLShoot.getPower() != lshootPower || motorRShoot.getPower() != rshootPower) {
                motorLShoot.setPower(lshootPower);
                motorRShoot.setPower(rshootPower);
                SystemClock.sleep(3000);
            }
            servoShooterGate.setPosition(shootgateMax);
            SystemClock.sleep(1000);
            servoShooterGate.setPosition(shootgateMin);
            SystemClock.sleep(1000);
            servoShooterPipe.setPosition(shootpipeMax);
            SystemClock.sleep(1000);
            servoShooterPipe.setPosition(shootgateMin);
            motorLShoot.setPower(0);
            motorRShoot.setPower(0);
        }
    }
}
