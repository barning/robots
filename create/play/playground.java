package play;

import org.jbox2d.common.Vec2;
import processing.core.PApplet;
import processing.core.PGraphics;
import de.hfkbremen.robots.challenge.*;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

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

        private final float STANDING = 0.5f;

        private float mAngle;

        private final Sensor mSensor_front;
        private final Sensor mSensor_left;
        private final Sensor mSensor_right;
        private final Sensor mSensor_back;

        private Vec2 old_position;
        private float angleSpeed = 3f;

        private int lastTime = 0;

        private float sensorFront;
        private boolean steerLeft = false;
        private boolean steerRight = false;

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);
            mSensor_front = addSensor(0, 40.0f);
            mSensor_left = addSensor(0, 40.0f);
            mSensor_right = addSensor(0, 40.0f);
            mSensor_back = addSensor(0, 40.0f);
        }

        public void update(float pDeltaTime) {

            mAngle += pDeltaTime * angleSpeed;
            mSensor_front.angle(mAngle);
            mSensor_back.angle(mAngle + PI);

            mSensor_left.angle(-(mAngle - PI/2));
            mSensor_right.angle(-(mAngle + PI/2));

            Vec2 position = this.position();

            speed(maxForwardSpeed/2);
            if (mSensor_front.triggered() && !steerRight && !steerLeft) {
                sensorFront = mSensor_front.angle()%(2*PI);
                println(sensorFront);
                if (sensorFront <= PI) {
                    steerRight = true;
                } else {
                    steerLeft = true;
                }

            } else {
                steer(0);
            }

            if (steerLeft) {
                steer(steer() - 0.9f);
                if( millis() - lastTime >= 3000){
                    steerLeft = false;
                }
            }
            if (steerRight) {
                steer(steer() + 0.9f);
                if( millis() - lastTime >= 3000){
                    steerRight = false;
                }
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

            old_position = position;

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

        private boolean isStanding(final Vec2 old_position, final Vec2 current_position) {
            if (old_position != null && current_position != null) {

                Vec2 velocity = current_position.sub(old_position);
                return velocity.length() <= STANDING;
            }
            return false;
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{playground.class.getName()});
    }
}
