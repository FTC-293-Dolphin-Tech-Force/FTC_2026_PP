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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.dtf_base_libraries.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.dtf_base_libraries.MecanumRobotControllerOld;
import org.firstinspires.ftc.teamcode.dtf_base_libraries.PinpointLocalizer;

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
@TeleOp
@Deprecated
public class OldTuner extends LinearOpMode { //this op mode used to be called PIDConstantTuner

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

        MecanumRobotControllerOld robot = new MecanumRobotControllerOld(hardwareMap, runtime, motorNames, reverseList, PIDList, dt, dimensions, localizer);

        double xt = 0, yt = 0, ht = 0;
        double x = 0, y = 0, h = 0;
        float kPval = (float) 0.105, kIval = (float) 0.0036, kDval = (float) 0.037;
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

            telemetry.addData("Target Position", "%4.3f, %4.3f, %4.3f", xt, yt, ht);
            VectorF targetPose = new VectorF((float)xt, (float)yt, (float) ht);
            if(gamepad1.a) {
                double error = targetPose.subtracted(robot.getLocalizer().getPose()).magnitude();
                telemetry.addData("error", "%4.3f", error);

                robot.setTargetPosition(targetPose, runtime.seconds());
            }
            else{
                robot.setTargetVelocity(new VectorF(0, 0, 0));
            }
            if(gamepad1.x){
                kPval+=0.01;
                robot.setkP(kPval);
            }
            if(gamepad1.dpad_left){
                kPval-=0.01;
                robot.setkP(kPval);
            }
            if(gamepad1.y){
                kIval+=0.0001;
                robot.setkI(kIval);
            }
            if(gamepad1.dpad_up){
                kIval-=0.0001;
                robot.setkI(kIval);
            }
            if(gamepad1.b){
                kDval+=0.001;
                robot.setkD(kDval);
            }
            if(gamepad1.dpad_right){
                kDval-=0.001;
                robot.setkD(kDval);
            }
            if(gamepad1.left_trigger>0.5){
                xt=0.6;
            }
            if(gamepad1.right_trigger>0.5){
                xt=-0.6;
            }
            for(int i = 0; i < 4; i++){
                if(robot.motors[i].getPower()>0.75){
                    for(int j = 0; j < 4; j++){
                        robot.motors[j].setPower(robot.motors[j].getPower()*0.75/robot.motors[i].getPower());
                    }
                }
                telemetry.addData("velocity rad/s, power /100", "%4.1f, %4.2f, %4.3f", (float)i, robot.getMotor(i).getVelocity(AngleUnit.RADIANS), robot.getMotor(i).getPower());
            }
            telemetry.addData("kP, kI, kD", "%4.3f, %4.5f, %4.4f", kPval, kIval, kDval);
            robot.getLocalizer().updatePose();
            x = robot.getLocalizer().getPose().get(0);
            y = robot.getLocalizer().getPose().get(1);
            h = robot.getLocalizer().getPose().get(2);
            telemetry.addData("x", "%4.3f", x);
            telemetry.addData("y", "%4.3f", y);
            telemetry.addData("pp heading", "%4.3f", h);
            telemetry.update();

            //robot.getLocalizer().updateTelemetry();



        }
    }
}
