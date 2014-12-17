package play;

import org.jbox2d.common.Vec2;
import processing.core.PApplet;
import processing.core.PGraphics;
import de.hfkbremen.robots.challenge.*;

/**
 *
 *
 * Created by felixkroll on 11.12.14.
 */
public class playground extends PApplet{

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

        private float mAngle;

        private final Sensor mSensor_front;
        private final Sensor mSensor_left;
        private final Sensor mSensor_right;
        private final Sensor mSensor_back;

        private float angleSpeed = 18f;
        private int lastTime = 0;

        private float sensorFrontAngle;
        //Ist weniger eine Zeit, als die Anzahl der Frames, in der bestimmte Aktionen ausgeführt werden soll.
        private int steeringTime = 60;
        private int backTime = 100;
        private Vec2 oldPosition;

        private boolean steerLeft = false;
        private boolean steerRight = false;
        private boolean driveBack = false;

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);
            mSensor_front = addSensor(0, 35.0f);
            mSensor_left = addSensor(0, 35.0f);
            mSensor_right = addSensor(0, 35.0f);
            mSensor_back = addSensor(0, 35.0f);
        }

        public void update(float pDeltaTime) {

            mAngle += pDeltaTime * angleSpeed;
            mSensor_front.angle(mAngle);
            mSensor_back.angle(mAngle + PI);

            mSensor_left.angle(-(mAngle - PI/2));
            mSensor_right.angle(-(mAngle + PI/2));

            speed(maxForwardSpeed);

            if (mSensor_front.triggered()) {
                sensorFrontAngle = mSensor_front.angle()%(2*PI);
                println(sensorFrontAngle);
                if (sensorFrontAngle <= PI/2 && !steerLeft) {
                    steerRight = true;
                    lastTime = steeringTime;
                } else if(sensorFrontAngle > 3*PI/2 && !steerRight) {
                    steerLeft = true;
                    lastTime = steeringTime;
                    //Wie kann man abfragen, ob der Roboter steht…
                } else if(oldPosition.sub(this.position()).length() == 0 && sensorFrontAngle <= PI/2 || sensorFrontAngle >= 3*PI/2 && !driveBack) {
                    driveBack = true;
                    lastTime = backTime;
                }
            }

            if (steerLeft) {
                steer(steer() - 0.9f);
                lastTime -= mSensor_front.triggered() ? 1 : 4;
                if(lastTime <= 0){
                    steerLeft = false;
                }
            }
            if (steerRight) {
                steer(steer() + 0.9f);
                lastTime -= mSensor_front.triggered() ? 1 : 4;
                if(lastTime <= 0){
                    steerRight = false;
                }
            }
            if (driveBack) {
                speed(maxBackwardSpeed);
                if (sensorFrontAngle >= PI && sensorFrontAngle <= 3*PI/2) {
                    steer(steer() + 0.9f);
                } else {
                    steer(steer() - 0.9f);
                }
                backTime -= 1;
                if(backTime <= 0){
                    driveBack = false;
                }
            }

            oldPosition = this.position();

            if (!steerLeft && !steerRight){
                steer(0);
            }

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
                        speed(50);
                        break;
                    case 's':
                        speed(0);
                        break;
                }
            }
        }

        public void draw(PGraphics g) {
        /* draw in robot s local coordinate space */
            rectMode(CENTER);
            noStroke();
            fill(0, 127, 255);
            if (!mSensor_front.triggered()) {
                rect(1.5f, -2, 2, 2);
            }
          /*  if (!mSensor_left.triggered()) {
                ellipse(-1.5f, -2, 2, 2);
            }*/
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{playground.class.getName()});
    }
}
