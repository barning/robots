package SteerRobotExample;
import processing.core.PGraphics;
import shiffman.box2d.*;
import de.hfkbremen.robots.challenge.*;
import processing.core.PApplet;

/**
 * SteerRobotExample
 *
 * Created by felixkroll on 11.12.14.
 */
public class example2 extends PApplet{

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

        final Sensor mSensorA;
        final Sensor mSensorB;
        final Sensor mSensorC;

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);
            mSensorA = addSensor(PI / 4, 20.0f);
            mSensorB = addSensor(PI / -4, 20.0f);
            mSensorC = addSensor(0, 20.0f);
        }

        public void update() {
        /* steer robot and controll its motor */
            if (keyPressed) {
                switch (key) {
                    case 'a':
                        steer(steer() + 0.05f);
                        break;
                    case 'd':
                        steer(steer() - 0.05f);
                        break;
                    case 'w':
                        speed(maxForwardSpeed);
                        break;
                    case 's':
                        speed(-maxForwardSpeed);
                        steer(0);
                        break;
                    case 'x':
                        speed(maxBackwardSpeed);
                        break;
                    case '-':
                        mScale -= 0.5f;
                        mScale = max(1, mScale);
                        break;
                    case '+':
                        mScale += 0.5f;
                        mScale = min(5, mScale);
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
                ellipse(-1.5f, -2, 2, 2);
            }
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{example2.class.getName()});
    }
}


