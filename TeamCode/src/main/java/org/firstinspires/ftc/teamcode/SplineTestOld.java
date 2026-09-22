/*
Copyright 2026 FIRST Tech Challenge Team 293

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

/*

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
//@TeleOp

//@Deprecated
/*
public class SplineTestOld extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    double dt = 0.02;

    //@Override
    public void runOpMode() {


        String[] motorNames = {"DriveLF", "DriveRF", "DriveLB", "DriveRB"};
        boolean[] reverseList = {true, false, true, false};
        double P, I, D, F;
        P = 2.5;
        I = 0;
        D = 0;
        F = 1.5*(435/12.0/2); //load factor * (max rpm / max voltage) * 1/2
        PIDFCoefficients[] PIDList = {
                new PIDFCoefficients(P, I, D, F),
                new PIDFCoefficients(P, I, D, F),
                new PIDFCoefficients(P, I, D, F),
                new PIDFCoefficients(P, I, D, F)};
        dt = 0.02;
        double[] dimensions = {0.15, 0.19125, 0.052}; //length, width, wheel radius, in meters

        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, runtime, new VectorF(0, 0, 0), new VectorF(0, 0, 0), 118, 126, GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD, new GoBildaPinpointDriver.EncoderDirection[]{GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED});

        MecanumRobotController robot = new MecanumRobotController(hardwareMap, runtime, motorNames, reverseList, PIDList, dt, dimensions, localizer);

        double xt = 0, yt = 0, ht = 0;
        double x = 0, y = 0, h = 0;
        int i = 0; int j = 0;
        double sensitivity = -2.0;
        float kPval = (float) 0.60, kIval = (float) 0, kDval = (float) 0.12;
        double timeNow = 0;

        SplineOld path = new SplineOld(new double[]{1, 0, -0.5, 0, 0.0417}, new double[]{0, 1, 0, -0.1667, 0, 0.0083}, new double[]{0});

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetRuntime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            xt += -0.08*gamepad1.left_stick_y;
            yt += -0.08*gamepad1.left_stick_x;
            ht += -0.08*gamepad1.right_stick_x;


            telemetry.addData("Target Pose", "%4.3f, %4.3f, %4.3f", xt, yt, ht);
            VectorF targetPose = new VectorF((float)xt, (float)yt, (float) ht);
            VectorF targetVel = robot.PID(targetPose, runtime.seconds());
            telemetry.addData("Target Vel", "%4.3f, %4.3f, %4.3f", targetVel.get(0), targetVel.get(1), targetVel.get(2));
            telemetry.addData("runtime.seconds", "%4.3f", runtime.seconds()-timeNow);
            if(gamepad1.dpad_down){
                timeNow = runtime.seconds();
            }
            if(gamepad1.a) {
                double error = targetPose.subtracted(robot.getLocalizer().getPose()).magnitude();
                telemetry.addData("error", "%4.3f", error);

                robot.followPath(path, (runtime.seconds()-timeNow));
            }
            else{
                robot.setTargetVelocity(new VectorF(0, 0, 0));
            }

            if(gamepad1.dpad_left){i = 0; telemetry.addData("Matrix:", "(P)roportional");}
            else if(gamepad1.dpad_up){i = 1; telemetry.addData("Matrix:", "(I)ntegral");}
            else if(gamepad1.dpad_right){i = 2; telemetry.addData("Matrix:", "(D)erivative");}

            if(gamepad1.x){j = 0; telemetry.addData("Axis:", "(x)");}
            else if(gamepad1.y){j = 1; telemetry.addData("Axis:", "(y)");}
            else if(gamepad1.b){j = 2; telemetry.addData("Axis:", "(h)eading");}

            telemetry.addData("Value:", "%4.4f", robot.getPIDconstant(i, j));

            if(gamepad1.right_trigger>0.5){
                robot.setPIDconstant(i, j, robot.getPIDconstant(i, j)+(float)Math.pow(10,sensitivity));
            }
            else if(gamepad1.left_trigger>0.5){
                robot.setPIDconstant(i, j, robot.getPIDconstant(i, j)-(float)Math.pow(10, sensitivity));
            }

            if(gamepad1.right_bumper){
                sensitivity+=1;
                if(sensitivity>=0){sensitivity=-1;}
            }
            else if(gamepad1.left_bumper){
                sensitivity+=-1;
                if(sensitivity<=-5){sensitivity=-4;}
            }
            telemetry.addData("Sensitivity:", "%4.2f", sensitivity);

            if(gamepad1.start){
                xt=0.6;
                yt = 0;
            }
            else if(gamepad1.back){
                xt=-0.6;
                yt = 0;
            }

            /*for(int m = 0; m < 4; m++){
                if(robot.motors[i].getPower()>0.75){
                    for(int n = 0; n < 4; n++){
                        robot.motors[j].setPower(robot.motors[j].getPower()*0.75/robot.motors[i].getPower());
                    }
                }
                telemetry.addData("velocity rad/s, power /100", "%4.1f, %4.2f, %4.3f", (float)i, robot.getMotor(i).getVelocity(AngleUnit.RADIANS), robot.getMotor(i).getPower());
            }*/
/*
            robot.getLocalizer().updatePose();
            x = robot.getLocalizer().getPose().get(0);
            y = robot.getLocalizer().getPose().get(1);
            h = robot.getLocalizer().getPose().get(2);
            telemetry.addData("True Pose (x, y, h)", "%4.3f, %4.3f, %4.3f", x, y, h);
            telemetry.addData("Target Derivative", "%4.3f, %4.3f, %4.3f", path.getdx(0.1*(runtime.seconds()-timeNow)), path.getdy(0.1*(runtime.seconds()-timeNow)), path.getdh(0.1*(runtime.seconds()-timeNow)));
            telemetry.update();

            //robot.getLocalizer().updateTelemetry();



        }
    }
}
*/