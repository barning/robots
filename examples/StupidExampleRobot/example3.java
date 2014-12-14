package StupidExampleRobot;

import processing.core.PApplet;
import shiffman.box2d.*;
import de.hfkbremen.robots.challenge.*;

/**
 * StupidExampleRobot
 *
 * Created by felixkroll on 11.12.14.
 */
public class example3 extends PApplet {
    Environment mEnvironment;
    MyRobot mRobot;
    float mScale = 5;

    public void setup() {
        size(1024, 768);
        mEnvironment = new Environment(this, Environment.MAP_BAELLEBAD);

        mRobot = new MyRobot(mEnvironment);
        mEnvironment.add(mRobot);
    }

    public void draw() {
        background(255);
        mEnvironment.update();
        mEnvironment.draw(g, mScale, mRobot.position());
    }

    class MyRobot extends Robot {

        final Sensor frontSensor;
        final Sensor backSensor;
        boolean obstacleFront = false;

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);
            frontSensor = addSensor(0, 50.0f);
            backSensor  = addSensor(PI, 50.0f);
        }

        public void update() {
            /* steer robot and controll its motor */
            if (frontSensor.triggered())
            {
                obstacleFront = true;
            } else if (backSensor.triggered())
            {
                obstacleFront = false;
            }
            if (obstacleFront)
                speed(maxBackwardSpeed);
            else
                speed(maxForwardSpeed);
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{example3.class.getName()});
    }
}
