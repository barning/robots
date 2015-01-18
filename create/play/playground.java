package play;

import org.jbox2d.common.Vec2;
import processing.core.PApplet;
import processing.core.PGraphics;
import de.hfkbremen.robots.challenge.*;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;


public class playground extends PApplet{

    Environment mEnvironment;
    MyRobot mRobot;
    float mScale = 5;
    Trace mTrace;
    OSD mOSD;

    public void setup() {
        size(1024, 768);
        mEnvironment = new Environment(this, Environment.MAP_BAELLEBAD);

        mRobot = new MyRobot(mEnvironment);
        mEnvironment.add(mRobot);

        mTrace = new Trace(mRobot);
        mOSD = new OSD(this, mEnvironment);
    }

    public void draw() {
        background(255);
        mEnvironment.update();

        mTrace.update(mEnvironment.delta_time());
        mEnvironment.beginDraw(g, mScale, mRobot.position());
        stroke(0);
        noFill();
        mTrace.draw(g);
        mEnvironment.endDraw(g);
        mOSD.draw(g);
    }

    /**
     * TODO nur nehmen, wenn es keinen anderen Weg mehr gibt
     *
     * Unsere Wahrnehmungsfelder als Enum.
     * Jeder Enum-Eintrag steht für ein Feld, in dem potentielle Hindernisse liegen können.
     *
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

    /**
     * Satus des Robots. Der Roboter kann jeweils nur FORWARD oder BACKWARD einnehmen und LEFT_DIRECTION oder RIGHT_DIRECTION
     */
    public enum ROBO_STATUS {
        FORWARD("forward"), BACKWARD("backward"), STANDING("standing"), LEFT_DIRECTION("left"), RIGHT_DIRECTION("right"),
        NO_STEERING("forward driving");

        private String name;

        ROBO_STATUS(final String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    class MyRobot extends Robot {

        //TODO Erstmal nicht
        /**
         * Repräsentiert den Messpunkt eines Sensors.
         */
        class Value {

            /**
             * Der Punkt in Weltkoordinaten, an dem das Objekt registriert wurde. Statisch!
             */
            private final Vec2 value_position;

            /**
             * Der Winkel relativ zur Roboterspitze.
             * Wird aktualisiert.
             */
            private float degree;

            /**
             * Der Abstand zum Robotermittlepunkt
             * Wird aktualisiert.
             */
            private float distance;

            /**
             * Initialisiert einen Messpunkt.
             *
             * Wirft eine {@link java.lang.IllegalArgumentException},
             * wenn {@code value_position} gleich {@code null} ist
             *
             * @param value_position Position zum Robotermittlepunkt zum Messzeitpunkt.
             * @param degree Winkel relativ zur Roboterspitze zum Messzeitpunkt
             * @param distance Abstand zum Robotermittlepunkt zum Messzeitpunkt
             */
            Value(final Vec2 value_position, final float degree, final float distance) {
                if (value_position == null) {
                    throw new IllegalArgumentException();
                }
                this.value_position = new Vec2(value_position);
                this.degree = degree;
                this.distance = distance;
            }

            /* Getter Methoden */
            public float getDegree() {
                return degree;
            }

            public Vec2 getValue_position() {
                return value_position;
            }

            public float getDistance() {
                return distance;
            }

            /*Setter Methoden*/
            public void setDegree(float degree) {
                this.degree = degree;
            }

            public void setDistance(float distance) {
                this.distance = distance;
            }

        }

        //TODO Erstmal nicht
        /**
         * Datenstruktur für ein Hinderniss.
         * Besteht aus einzelen Messpunkten, die in einer ArrayList gespeichert werden.
         * Ein neuer Messpunkt, der in der Nähe eines schon in der ArrayList befindlichen Messpunktes ist,
         * wird an die richtige Position in der ArrayList gestellt.
         *
         * Zwei Hindernisse werde zu einem, wenn ihre Enden aneinander liegen. TODO
         */
        class Obstacle {

            /**
             * Maximale Distanz, bei der ein Messpunkt noch hinzugefügt wird. Inklusiv.
             * TODO ein passender Wert muss noch gefunden werden.
             */
            private final float MAX_DISTANCE_TO_NEXT = 0.1f;

            /**
             * Minimale Distanz, bei der ein Messpunkt nicht mehr hinzugefügt wird. Inklusiv.
             * TODO ein passender Wert muss noch gefunden werden.
             */
            private final float MIN_DISTANCE_TO_NEXT = 0.05f;

            /**
             * Der Mittelpunkt des Hindernisses in Weltkoordinaten.
             * Wir immer aktualisiert, wenn sich das Objekt ändert.
             */
            private Vec2 position;

            private final ArrayList<Value> values;

            /**
             * Initialisiert ein Hinderniss.
             */
            Obstacle() {
                values = new ArrayList<>();
            }

            /**
             * Fügt ein neuen Messpunkt in die ArrayList ein. Ein neuer Messpunkt wird nur dann in die ArrayList eingefügt,
             * wenn zu einem bereits enthalten Messpunkt den Abstand MAX_DISTANCE_TO_NEXT und MIN_DISTANCE_TO_NEXT einhält.
             *
             * Wenn die ArrayList noch leer ist, wird das Objekt einfach hinzugefügt.
             *
             * @param value Messpunkt, der ein gefügt werden soll. Darf nicht {@code null} sein.
             *              Sonst wird eine {@link java.lang.IllegalArgumentException} geworfen.
             * @return
             *      Wenn der Punkt auserhalb von MAX_DISTANCE_TO_NEXT liegt,
             *      wird -1 zurückgegeben.
             *      In diesem Fall kann nach ausführen dieser Mehtode ein neues Hinderniss erzeugt werden.
             *
             *      Wenn der Punkt auserhalb von MIN_DISTANCE_TO_NEXT liegt,
             *      wird 0 zurückgegeben.
             *
             *      Wenn der Punkt eingefügt werden konnte, wird 1 zurückgegeben.
             */
            public int addValue(final Value value) {
                if (value == null) throw  new IllegalArgumentException();

                if (values.isEmpty()) {
                    values.add(value);
                    position = value.getValue_position();
                    return 1;
                }


                //Aus Geschwindigkeitsgründen nur für erstes und letztes Element.

                //Prüfung für das erste Element
                float length = value.getValue_position().sub(values.get(0).getValue_position()).lengthSquared();

                if (length >= MAX_DISTANCE_TO_NEXT*MAX_DISTANCE_TO_NEXT) {
                    return -1;
                } else if (length > MIN_DISTANCE_TO_NEXT*MIN_DISTANCE_TO_NEXT) {
                    values.add(0, value);
                    //TODO calculate position
                    return 1;

                } else {
                    //Prüfung für das letze Element
                    length = value.getValue_position().sub(values.get(values.size()-1).getValue_position()).lengthSquared();

                    if (length >= MAX_DISTANCE_TO_NEXT*MAX_DISTANCE_TO_NEXT) {
                        return -1;
                    } else if (length > MIN_DISTANCE_TO_NEXT*MIN_DISTANCE_TO_NEXT) {
                        values.add(value);
                        //TODO calculate position
                        return 1;
                    } else {
                        return 0;
                    }
                }
            }

            /**
             * Liegen Datenpunkte von zwei Hindernissen in unmittlebarer Nähe, werden sie zu einem Hinderniss zusammen gefügt.
             */
            public void mergeObstacles() {

            }

            /**
             * aktualisiert alle Values in Obstacle. Winkel wird neu berechnet und Länge.
             * TODO geht das auch nebenläufig ???
             */
            public void updateObstacle(final Robot robot) {
                /*
                for(Value value : values) {
                    value.setDistance(
                            value.getValue_position().sub(robot.position()).length()
                    );
                    value.setDegree(
                        //TODO
                    );
                }
                */
            }

            /**
             * Zeichnet das Obstacle in die Welt. //TODO Nur in draw-global möglich ?
             */
            public void drawObstacle() {
                noStroke();
                fill(255,50,0);

                for (Value v : values) {
                    Vec2 pos = v.getValue_position();
                    ellipse(pos.x, pos.y, 2, 2);
                }
            }

            /**
             * Gibt eine Kopie der ArrayList zurück.
             *
             * @return Kopie der ArrayList
             */
            public ArrayList<Value> GetValues() {
                return new ArrayList<>(values);
            }

        }

        /**
         * Container für nützliche Daten über den Roboter aus dem letzten Frame
         */
        class LastRobo {

            private float steer;

            private float speed;

            public float getSteer() {
                return steer;
            }

            public float getSpeed() {
                return speed;
            }

            public Vec2 getPosition() {
                return lastPosition;
            }

            public float[][] getLastSensorData() {
                return lastSensorData;
            }

            private Vec2 lastPosition;

            private float[][] lastSensorData = new float[5][4];

            public LastRobo(final float steer, final float speed, final Vec2 lastPosition, final Sensor[] sensors) {
                if (lastPosition == null || sensors == null) {
                    throw new IllegalArgumentException();
                }
                this.steer = steer;
                this.speed = speed;
                this.lastPosition = new Vec2 (lastPosition);

                for (int i = 0; i < 5; i++) {
                    for (int j = 0; j < 4; j++) {
                        switch (j) {
                            case 0:
                                lastSensorData[i][j] = sensors[i].angle();
                                break;
                            case 1:
                                lastSensorData[i][j] = sensors[i].obstacleDistance();
                                break;
                            case 2:
                                lastSensorData[i][j] = sensors[i].obstacle().x;
                                break;
                            case 3:
                                lastSensorData[i][j] = sensors[i].obstacle().y;
                                break;
                        }
                    }
                }
            }
        }

        /**
         * RingBuffer für Roboter Daten. Hat feste Größe. NUR add verwenden!!!, wenn man hinzufügen möchte.
         */
        class RingBuffer <E> extends ArrayDeque<E>{

            private final int maxCapacity;

            public RingBuffer(final int maxCapacity) {
                super(maxCapacity);
                if (maxCapacity <= 0) throw new IllegalArgumentException();
                this.maxCapacity = maxCapacity;
            }

            @Override
            public void push(E e) {
                super.push(e);
                if (size() > maxCapacity) {
                    removeLast();
                }
            }
        }

        /**
         * Winkel für Drehung der einzelnen Sensoren
         */
        private float mAngle;

        /**
         * Speichert, ob der Roboter links (LEFT_DIRECTION) oder rechts(RIGHT_DIRECTION) einlenkt.
         * Andere Stati sind nicht zulässig.
         */
        private ROBO_STATUS direction;

        /**
         * Speichert den Bewegungsstatus FORWARD, BACKWARD, STANDING.
         * Andere Stati sind nicht zulässig.
         */
        private ROBO_STATUS status;

        /**
         * Sensoren des Roboters
         */
        private final Sensor[] sensors;

        /**
         * Größe für lastFrameRobos. Entspricht drei Frames
         */
        private int LAST_ROBO_SIZE = 4;

        /**
         * Die letzten drei Roboter aus den letzten drei Frames.
         */
        private RingBuffer<LastRobo> lastRobos;

        /**
         * Epsilon für das isStanding-Prädikat
         */
        private final float EPS = 0.001f;

        /**
         * Deltasteer angle for tentacle movement
         */
        private final float TENTACLE_STEERING = 7*PI/4;

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);

            sensors = new Sensor[5];
            lastRobos = new RingBuffer<>(LAST_ROBO_SIZE);


            //front
            sensors[0] = addSensor(0      , maxSensorRange);
            //left
            sensors[1] = addSensor(- PI/2 , maxSensorRange);
            //right
            sensors[2] = addSensor(PI/2   , maxSensorRange);
            //back
            sensors[3] = addSensor(PI     , maxSensorRange);
            //tentacle
            sensors[4] = addSensor(0      , maxSensorRange);

            direction = ROBO_STATUS.NO_STEERING;
            status = ROBO_STATUS.FORWARD;
        }

        public void update(float pDeltaTime) {

            mAngle += pDeltaTime * 2;

            switch (direction) {
                case LEFT_DIRECTION:
                    sensors[0].angle(-(mAngle));
                    sensors[1].angle(-(mAngle + PI));
                    sensors[2].angle(-(mAngle + PI / 2));
                    sensors[3].angle(-(mAngle + 3 * PI / 2));
                    break;
                case RIGHT_DIRECTION:
                    sensors[0].angle(mAngle);
                    sensors[1].angle(mAngle + PI);
                    sensors[2].angle(mAngle + PI / 2);
                    sensors[3].angle(mAngle + 3 * PI / 2);
                    break;
                case NO_STEERING:
                    sensors[0].angle(-(mAngle));
                    sensors[1].angle(-(mAngle + PI));
                    sensors[2].angle(-(mAngle + PI / 2));
                    sensors[3].angle(-(mAngle + 3 * PI / 2));
                    break;
            }

            switch (status) {
                case FORWARD:
                    sensors[4].angle(sin(mAngle) * 1/2 - steer());
                    break;
                case BACKWARD:
                    sensors[4].angle(sin(mAngle) * 1/2 - steer() + PI);
                    break;
                default:
                    sensors[4].angle(sin(mAngle) * 1/2);
                    break;
            }


            boolean obstacleInView = sensors[0].triggered() || sensors[1].triggered()
                                  || sensors[2].triggered() || sensors[3].triggered()
                                  || sensors[4].triggered();

            if (obstacleInView) {

                Sensor sensor;
                switch (status) {
                    case FORWARD:
                        sensor = mostDisturbingFrontObstacle(sensors);

                        if (sensor != null) {
                            speed(maxForwardSpeed * sensor.obstacleDistance() * 0.2f);

                            float steeringAngle = sensor.angle()%(2*PI);

                            if (sensor.obstacleDistance() < 0.90f) {

                                if (steeringAngle > -3*PI/2) {
                                    steer(-maxSteeringAngle);
                                } else {
                                    steer(maxSteeringAngle);
                                }
                            }
                        }

                        break;
                    case BACKWARD:
                        sensor = mostDisturbingFrontObstacle(sensors);
                        break;
                }

            } else {

                speed(maxForwardSpeed);
                float steeringAngle = angleToGoal();
                steer(steeringAngle);
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
                        steer(0);
                        break;
                    case  'x':
                        speed(maxBackwardSpeed);
                        break;
                }
            }

            //Save LastFrameRobo -> Before State Update !!!
            lastRobos.push(new LastRobo(steer(), speed(), position(), sensors));
            //Update States
            updateStates();

        }

        private void updateStates() {

            float middleVelocity = 0f;
            int countLeftDirection = 0;
            int countRightDirection = 0;

            Iterator<LastRobo> iter = lastRobos.iterator();
            LastRobo lastRobo = iter.hasNext() ? iter.next() : null;
            while (iter.hasNext() && lastRobo != null) {
                LastRobo newerRobo = iter.next();

                middleVelocity += newerRobo.getPosition().sub(lastRobo.getPosition()).length();

                lastRobo = newerRobo;
            }
            middleVelocity /= lastRobos.size();

            //Status test
            if (-EPS < middleVelocity && middleVelocity < EPS) {
                status = ROBO_STATUS.STANDING;
            } else if(middleVelocity <= -EPS) {
                status = ROBO_STATUS.BACKWARD;
            } else {
                status = ROBO_STATUS.FORWARD;
            }


            for (LastRobo robo : lastRobos) {
                if (robo.getSteer() <= 0) {
                    countLeftDirection++;
                } else {
                    countRightDirection++;
                }
            }

            //Direction test
            if (countLeftDirection > countRightDirection) {
                direction = ROBO_STATUS.LEFT_DIRECTION;
            } else if (countLeftDirection < countRightDirection) {
                direction = ROBO_STATUS.RIGHT_DIRECTION;
            } else {
                direction = ROBO_STATUS.NO_STEERING;
            }

        }

        private Sensor mostDisturbingFrontObstacle(final Sensor[] sensors) {
            if (sensors == null) throw  new IllegalArgumentException();

            Sensor mostImportantSensor = null;
            float distanceToObstacle = MAX_FLOAT;
            float angleToObstacle = PI; // Ab der Hälfte muss getestet werden

            for (Sensor s : sensors) {
                if (s.triggered()) {
                    float angleToTest = s.angle() % (2 * PI);

                    //Ist der Winkel überhaupt relevant ?
                    if (angleToTest >= PI/2 && angleToTest <= 3*PI/2) {
                        continue;
                    }

                    if (angleToTest > 3*PI/2) {
                        angleToTest = 2*PI - angleToTest;
                    }

                    if (s.obstacleDistance() < distanceToObstacle && angleToTest < angleToObstacle) {
                        distanceToObstacle = s.obstacleDistance();
                        angleToObstacle = angleToTest;
                        mostImportantSensor = s;
                    }
                }
            }
            return mostImportantSensor;
        }

        private Sensor mostDisturbingBackObstacle(final Sensor[] sensors) {

            Sensor mostImportantSensor = null;
            float distanceToObstacle = MAX_FLOAT;
            float angleToObstacle = 2*PI; // Ab der Hälfte muss getestet werden

            for (Sensor s : sensors) {
                if (s.triggered()) {
                    float angleToTest = s.angle() % (2 * PI);

                    //Ist der Winkel überhaupt relevant ?
                    if (angleToTest < PI/2 && angleToTest > 3*PI/2) {
                        continue;
                    }

                    if (angleToTest > PI) {
                        angleToTest = angleToTest - PI;
                    }

                    if (angleToTest <= PI) {
                        angleToTest = PI - angleToTest;
                    }

                    if (s.obstacleDistance() < distanceToObstacle && angleToTest < angleToObstacle) {
                        distanceToObstacle = s.obstacleDistance();
                        angleToObstacle = angleToTest;
                        mostImportantSensor = s;
                    }
                }
            }
            return mostImportantSensor;
        }

        public void draw(PGraphics g) {
        /* draw in robot s local coordinate space */
            rectMode(CENTER);
            noStroke();
            fill(0, 127, 255);


            switch (direction) {
                case LEFT_DIRECTION:
                    triangle(2, 2, 4, 0, 2, -2);
                    break;
                case RIGHT_DIRECTION:
                    triangle(-2, 2, -4, 0, -2, -2);
                    break;
                case NO_STEERING:
                    //Draw Nothing
                    break;
            }

            switch (status) {
                case FORWARD:
                    triangle(2, 2, 0, 4, -2, 2);
                    break;
                case BACKWARD:
                    triangle(2, -2, 0, -4, -2, -2);
                    break;
                default:
                    //Draw Nothing
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
