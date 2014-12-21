package play;

import org.jbox2d.common.Vec2;
import processing.core.PApplet;
import processing.core.PGraphics;
import de.hfkbremen.robots.challenge.*;


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

    /**
     * Unsere Wahrnehmungsfelder als Enum.
     * Jeder Enum-Eintrag steht für ein Feld, in dem potentielle Hindernisse liegen können.
     */
    public enum DetectionField {

        /*
            Hier sind die äußeren Felder aufgelistet
        */
        OUTER_CIRCLE_0(0, PI/6, 1, 0.7f),           OUTER_CIRCLE_1(PI/6, PI/3, 1, 0.7f),
        OUTER_CIRCLE_2(PI/3, PI/2, 1, 0.7f),        OUTER_CIRCLE_3(PI/2, 2*PI/3, 1, 0.7f),
        OUTER_CIRCLE_4(2*PI/3, PI, 1, 0.7f),        OUTER_CIRCLE_5(PI, 4*PI/3, 1, 0.7f),
        OUTER_CIRCLE_6(4*PI/3, 3*PI/2, 1, 0.7f),    OUTER_CIRCLE_7(3*PI/3, 11*PI/6, 1, 0.7f),
        OUTER_CIRCLE_8(11*PI/6, 2*PI, 1, 0.7f),

        /*
            Hier sind die mittleren Felder aufgelistet
        */
        MIDDLE_CIRCLE_0(0, PI/6, 0.7f, 0.4f),          MIDDLE_CIRCLE_1(PI/6, PI/3, 0.7f, 0.4f),
        MIDDLE_CIRCLE_2(PI/3, PI/2, 0.7f, 0.4f),       MIDDLE_CIRCLE_3(PI/2, 2*PI/3, 0.7f, 0.4f),
        MIDDLE_CIRCLE_4(2*PI/3, PI, 0.7f, 0.4f),       MIDDLE_CIRCLE_5(PI, 4*PI/3, 0.7f, 0.4f),
        MIDDLE_CIRCLE_6(4*PI/3, 3*PI/2, 0.7f, 0.4f),   MIDDLE_CIRCLE_7(3*PI/3, 11*PI/6, 0.7f, 0.4f),
        MIDDLE_CIRCLE_8(11*PI/6, 2*PI, 0.7f, 0.4f),

        /*
            Hier sind die inneren Felder aufgelistet
        */
        INNER_CIRCLE_0(0, PI/6, 0.4f, 0),           INNER_CIRCLE_1(PI/6, PI/3, 0.4f, 0),
        INNER_CIRCLE_2(PI/3, PI/2, 0.4f, 0),        INNER_CIRCLE_3(PI/2, 2*PI/3, 0.4f, 0),
        INNER_CIRCLE_4(2*PI/3, PI, 0.4f, 0),        INNER_CIRCLE_5(PI, 4*PI/3, 0.4f, 0),
        INNER_CIRCLE_6(4*PI/3, 3*PI/2, 0.4f, 0),    INNER_CIRCLE_7(3*PI/3, 11*PI/6, 0.4f, 0),
        INNER_CIRCLE_8(11*PI/6, 2*PI, 0.4f, 0);

        /**
         * Im Bereich 0 bis 2PI
         */
        private final float startRad;

        /**
         * Im Bereich 0 bis 2PI
         */

        private final float endRad;

        /**
         * Im Bereich 1 bis 0
         */
        private final float startLength;

        /**
         * Im Bereich 1 bis 0
         */
        private final float endLength;

        /* Der GetterBlock */

        public float getStartRad() {
            return startRad;
        }

        public float getEndRad() {
            return endRad;
        }

        public float getStartLength() {
            return startLength;
        }

        public float getEndLength() {
            return endLength;
        }

        /**
         * Konstruktor für ein DetectionField
         *
         * @param startRad zwischen 0 und 2PI
         * @param endRad zwischen 0 und 2PI
         * @param startLength zwischen 1 und 0
         * @param endLength zwischen 1 und 0
         */
        DetectionField(final float startRad, final float endRad, final float startLength, final float endLength) {
            this.startRad = startRad;
            this.endRad = endRad;
            this.startLength = startLength;
            this.endLength = endLength;
        }

        /**
         * Liefert für Sensorwinkle und Hindernisslänge das passende Wahrnehmungsfeld wieder.
         *
         * @param rad Sensorwinkle
         * @param length Hindernisslänge
         * @return Wahrnehmungsfeld. Wenn kein passendes Feld gefunden werden kann, wir {@code null} zurückgegeben.
         */
        public static DetectionField selectField(final float rad, final float length) {
            for (DetectionField f  : DetectionField.values()) {
                if (rad >= f.startRad && rad <= f.endRad && length >= f.startLength && length <= f.endLength) {
                    return f;
                }
            }
            return null;
        }
    }

    class MyRobot extends Robot {

        private float mAngle;

        private final Sensor mSensor_front;
        private final Sensor mSensor_left;
        private final Sensor mSensor_right;
        private final Sensor mSensor_back;
        private final Sensor mSensor_tentacle;

        /*
        private final float EPS = 0.1f;
        private int lastTime = 0;
        private float sensorFrontAngle;
        //Ist weniger eine Zeit, als die Anzahl der Frames, in der bestimmte Aktionen ausgeführt werden soll.
        private int steeringTime = 60;
        private int backTime = 100;
        private Vec2 oldPosition;
        private boolean steerLeft = false;
        private boolean steerRight = false;
        private boolean driveBack = false;
        */

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);

            mSensor_front = addSensor(0, maxSensorRange);
            mSensor_left = addSensor(- PI/2, maxSensorRange);
            mSensor_right = addSensor(PI/2, maxSensorRange);
            mSensor_back = addSensor(PI, maxSensorRange);
            mSensor_tentacle = addSensor(0, 10.0f);
        }

        public void update(float pDeltaTime) {

            mAngle += pDeltaTime * 2;
            mSensor_front.angle(mAngle);
            mSensor_back.angle(mAngle + PI);
            mSensor_left.angle(mAngle + PI/2);
            mSensor_right.angle(mAngle + 3*PI/2);

            DetectionField field = DetectionField.selectField(0,0);


            if (field != null) {

            } else {
                speed(maxForwardSpeed);
                steer(angleToGoal());
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
                        speed(maxBackwardSpeed);
                        break;
                }
            }

            /*
            OLD CODE
            speed(maxForwardSpeed/2);

            if (mSensor_front.triggered()) {
                sensorFrontAngle = mSensor_front.angle()%(2*PI);

                println(sensorFrontAngle);

                if (sensorFrontAngle <= PI/2 && !steerLeft) {
                    steerRight = true;
                    lastTime = steeringTime;
                } else if(sensorFrontAngle > 3*PI/2 && !steerRight) {
                    steerLeft = true;
                    lastTime = steeringTime;

                } else if(oldPosition.sub(this.position()).length() <= EPS && (sensorFrontAngle <= PI/2 || sensorFrontAngle >= 3*PI/2) && !driveBack) {
                    driveBack = true;
                    lastTime = backTime;
                }
            }

            if (steerLeft) {
                steer(steer() - 0.9f);
                speed(maxForwardSpeed/4);
                lastTime -= mSensor_front.triggered() ? 1 : 4;
                if(lastTime <= 0){
                    steerLeft = false;
                }
            }
            if (steerRight) {
                steer(steer() + 0.9f);
                speed(maxForwardSpeed/4);
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
            */
        }

        public void draw(PGraphics g) {
        /* draw in robot s local coordinate space */
            rectMode(CENTER);
            noStroke();
            fill(0, 127, 255);
            if (!mSensor_front.triggered()) {
                rect(1.5f, -2, 2, 2);
            }
            if (!mSensor_left.triggered()) {
                ellipse(-1.5f, -2, 2, 2);
            }

            stroke(0);
            //draw a line pointing towards the goal
            float angle = angleToGoal() + PI / 2;
            float distance = distanceToGoal();

            Vec2 mGoalPosition = new Vec2(cos(angle), sin(angle));
            mGoalPosition.mulLocal(5);
            stroke(255, 127, 0);
            line(0, 0, mGoalPosition.x, mGoalPosition.y);
        }

        public void draw_global(PGraphics g) {
            // draw in the environment s coordinate space
            stroke(30);
            Vec2 target = mEnvironment.target().position();
            line(target.x, target.y, position().x, position().y);
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{playground.class.getName()});
    }
}
