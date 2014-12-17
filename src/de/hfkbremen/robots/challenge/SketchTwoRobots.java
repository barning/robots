package de.hfkbremen.robots.challenge;

import static processing.core.PConstants.PI;

import org.jbox2d.common.Vec2;
import processing.core.PApplet;
import processing.core.PGraphics;

import static processing.core.PConstants.CENTER;

public class SketchTwoRobots extends PApplet {

    private Environment mEnvironment;
    private MyRobot mRobot;
    private MyOtherRobot mOtherRobot;

    public void setup() {
        size(1024, 768);
        mEnvironment = new Environment(this, Environment.MAP_SIMPLE);

        mRobot = new MyRobot(mEnvironment);
        mEnvironment.add(mRobot);

        mOtherRobot = new MyOtherRobot(mEnvironment);
        mOtherRobot.body().setTransform(new Vec2(50, 20), 0);
        mEnvironment.add(mOtherRobot);
    }

    public void draw() {
        background(255);
        mEnvironment.update();
        mEnvironment.beginDraw(g);

        if (mRobot.mSensorA.triggered()) {
            drawObstacle(mRobot, mRobot.mSensorA);
        }
        if (mRobot.mSensorB.triggered()) {
            drawObstacle(mRobot, mRobot.mSensorB);
        }

        drawObstacle(mOtherRobot, mOtherRobot.mSensorA);
        drawObstacle(mOtherRobot, mOtherRobot.mSensorB);
        drawObstacle(mOtherRobot, mOtherRobot.mSensorC);

        mEnvironment.endDraw(g);
    }

    public void mousePressed() {
        new Wall(mEnvironment.world(), mouseX, mouseY, 20, 20);
    }

    private void drawObstacle(Robot pRobot, Sensor pSensor) {
        Vec2 mObstacle = pSensor.obstacle();
        if (pSensor.triggered()) {
            noStroke();
            fill(255, 127, 0);
        } else {
            noFill();
            stroke(255, 127, 0);
        }
        ellipse(mObstacle.x, mObstacle.y, 2, 2);
        noFill();
        stroke(255, 127, 0);
        line(pRobot.position(), pSensor.position());
    }

    private void line(Vec2 p1, Vec2 p2) {
        line(p1.x, p1.y, p2.x, p2.y);
    }

    public class MyRobot extends Robot {

        private final Sensor mSensorA;
        private final Sensor mSensorB;

        public MyRobot(Environment pEnvironment) {
            super(pEnvironment);
            mSensorA = addSensor(PI / 4, 14.0f);
            mSensorB = addSensor(PI / -4, 14.0f);
        }

        public void update(float pDelaTime) {
            /* steer robot and controll its motor */
            if (keyPressed) {
                switch (key) {
                    case 'a':
                        steer(steer() + 0.1f);
                        break;
                    case 'd':
                        steer(steer() - 0.1f);
                        break;
                    case 'w':
                        speed(speed() + 1);
                        break;
                    case 's':
                        speed(0);
                        steer(0);
                        break;
                    case 'x':
                        speed(speed() - 1);
                        break;
                }
            }
        }

        public void draw(PGraphics g) {
            /* draw in robot s local coordinate space */
            rectMode(CENTER);
            noStroke();
            fill(0, 127, 255);
            if (!mSensorA.triggered()) {
                rect(1.5f, -2, 2, 2);
            }
            if (!mSensorB.triggered()) {
                ellipse(1.5f, -2, 2, 2);
            }
        }
    }

    public class MyOtherRobot extends Robot {

        private final Sensor mSensorA;
        private final Sensor mSensorB;
        private final Sensor mSensorC;

        public MyOtherRobot(Environment pEnvironment) {
            super(pEnvironment);
            mSensorA = addSensor(PI / 4, 14.0f);
            mSensorB = addSensor(PI / -4, 14.0f);
            mSensorC = addSensor(PI, 14.0f);
        }

        public void update(float pDelaTime) {
            /* steer robot and controll its motor */
            if (keyPressed) {
                switch (key) {
                    case 'j':
                        steer(steer() + 0.1f);
                        break;
                    case 'l':
                        steer(steer() - 0.1f);
                        break;
                    case 'i':
                        speed(speed() + 1);
                        break;
                    case 'k':
                        speed(0);
                        steer(0);
                        break;
                    case ',':
                        speed(speed() - 1);
                        break;
                }
            }
        }

        public void draw(PGraphics g) {
            /* draw in robot s local coordinate space */
            rectMode(CENTER);
            noStroke();
            fill(0, 255, 127);
            if (!mSensorA.triggered()) {
                rect(1.5f, -2, 2, 2);
            }
            if (!mSensorB.triggered()) {
                ellipse(1.5f, -2, 2, 2);
            }
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{SketchTwoRobots.class.getName()});
    }
}
