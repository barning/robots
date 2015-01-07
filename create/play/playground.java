package play;

import org.jbox2d.common.Vec2;
import processing.core.PApplet;
import processing.core.PGraphics;
import de.hfkbremen.robots.challenge.*;

import java.util.ArrayList;


public class playground extends PApplet{

    Environment mEnvironment;
    MyRobot mRobot;
    float mScale = 5;

    public void setup() {
        size(1024, 768);
        mEnvironment = new Environment(this, Environment.MAP_RANDOM_WALLS);

        mRobot = new MyRobot(mEnvironment);
        mEnvironment.add(mRobot);
    }

    public void draw() {
        background(255);
        mEnvironment.update();
        mEnvironment.draw(g, mScale, mRobot.position());
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
     * Satus des Robots. Der Roboter kann jeweils nur FORWARD oder BACKWARD einnehmen und LEFT_DRIVING oder RIGHT_DRIVING
     */
    public enum ROBO_STATUS {
        FORWARD, BACKWARD, STANDING, LEFT_DRIVING, RIGHT_DRIVING
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

        private float mAngle;

        //TODO Erstmal nicht
        /**
         * Enthält alle gemessenen Hindernisse.
         */
        private final ArrayList<Obstacle> obstacles;

        /**
         * Speichert, ob der Roboter links (LEFT_DRIVING) oder rechts(RIGHT_DRIVING) einlenkt.
         * Andere Stati sind nicht zulässig.
         */
        private ROBO_STATUS direction;

        /**
         * Speichert den Bewegungsstatus FORWARD, BACKWARD, STANDING.
         * Andere Stati sind nicht zulässig.
         */
        private ROBO_STATUS status;

        private final Sensor[] sensors;

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);

            obstacles = new ArrayList<>();
            sensors = new Sensor[5];

            //front
            sensors[0] = addSensor(0      , maxSensorRange);
            //left
            sensors[1] = addSensor(- PI/2 , maxSensorRange);
            //right
            sensors[2] = addSensor(PI/2   , maxSensorRange);
            //back
            sensors[3] = addSensor(PI     , maxSensorRange);
            //tentacle
            sensors[4] = addSensor(0      , 10.0f);

            direction = ROBO_STATUS.LEFT_DRIVING;
            status = ROBO_STATUS.BACKWARD.FORWARD;
        }

        public void update(float pDeltaTime) {

            mAngle += pDeltaTime * 2;

            //Sonsor-Rotationsanpassung. Je nach Richtung, in die sich der Roboter bewegen soll.
            switch (direction) {
                case LEFT_DRIVING:
                    sensors[0].angle( -(mAngle)          );
                    sensors[1].angle ( -(mAngle + PI)     );
                    sensors[2].angle ( -(mAngle + PI/2)   );
                    sensors[3].angle( -(mAngle + 3*PI/2) );
                    break;
                case RIGHT_DRIVING:
                    sensors[0].angle( mAngle          );
                    sensors[1].angle ( mAngle + PI     );
                    sensors[2].angle ( mAngle + PI/2   );
                    sensors[3].angle( mAngle + 3*PI/2 );
                    break;
            }

            //TODO

            /*
            if (false) {

            } else {
                speed(maxForwardSpeed);
                steer(angleToGoal());
            } */


            /* steer robot and controll its motor */
            if (keyPressed) {
                switch (key) {
                    case 'a':
                        steer(steer() + 0.1f);
                        direction = ROBO_STATUS.LEFT_DRIVING;
                        break;
                    case 'd':
                        steer(steer() - 0.1f);
                        direction = ROBO_STATUS.RIGHT_DRIVING;
                        break;
                    case 'w':
                        speed(50);
                        status = ROBO_STATUS.FORWARD;
                        break;
                    case 's':
                        speed(maxBackwardSpeed);
                        status = ROBO_STATUS.BACKWARD;
                        break;
                }
            }


        }

        //TODO erstmal nicht verwenden
        /**
         * Fügt ein Messpunkt zu den beakannten Hindernissen hinzu.
         *
         * @param value Ein Messpunkt eines Sensors, wenn er ausgelöst wurde.
         */
        public void addObstacle(final Value value) {
            if (value == null) throw new IllegalArgumentException();

            if (obstacles.isEmpty()) {
                //Neues Obstacle wird erzeugt und Value wird hinzugefügt
                Obstacle obstacle = new Obstacle();
                obstacle.addValue(value);
                //nur Obstacle übergeben
                obstacles.add(obstacle);
                return;
            }

            int countToNewObstacle = 0;
            for (Obstacle o : obstacles) {
                switch (o.addValue(value)) {
                    case 1:
                        return;
                    case -1:
                       countToNewObstacle++;

                    //Punkte die zwischen sehr nah an bereitsbekannten Messwerten liegen Spiele hier keine Rolle mehr
                    //Deswegen wird hier Fall 0 nicht beachtet.
                }
            }
            //Neues Hindernis erzeugen, wenn der Messpunkt zu keine Hindernis zugeordnet werden kann.
            if (countToNewObstacle == obstacles.size()) {
                Obstacle obstacle = new Obstacle();
                obstacle.addValue(value);
                obstacles.add(obstacle);
            }
        }

        public void draw(PGraphics g) {
        /* draw in robot s local coordinate space */
            rectMode(CENTER);
            noStroke();
            fill(0, 127, 255);
            if (!sensors[0].triggered()) {
                rect(1.5f, -2, 2, 2);
            }
            if (!sensors[1].triggered()) {
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
