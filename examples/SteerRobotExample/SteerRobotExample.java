package SteerRobotExample;

import org.jbox2d.common.Vec2;
import processing.core.PApplet;
import processing.core.PGraphics;
import de.hfkbremen.robots.challenge.*;

/**
 * Created by niklasbarning on 14.12.14.
 */
public class SteerRobotExample extends PApplet{

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
        float mAngle;

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);
            mSensorA = addSensor(PI / 4, 20.0f);
            mSensorB = addSensor(PI / -4, 20.0f);
            mSensorC = addSensor(0, 20.0f);
        }

        public void update(float pDeltaTime) {
            mAngle += pDeltaTime;
            mSensorC.angle(sin(mAngle * 2) * 0.5f);

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
                        speed(0);
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
        PApplet.main(new String[]{SteerRobotExample.class.getName()});
    }

}
