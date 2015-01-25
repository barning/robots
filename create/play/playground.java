package play;

import de.hfkbremen.robots.challenge.Sensor;
import org.jbox2d.common.Vec2;
import processing.core.PApplet;
import processing.core.PGraphics;
import de.hfkbremen.robots.challenge.*;
import sun.management.*;

import java.util.*;


public class playground extends PApplet{

    Environment mEnvironment;
    MyRobot mRobot;
    float mScale = 5f;
    Trace mTrace;
    OSD mOSD;

    public void setup() {
        size(1024, 768);

        /*
        mEnvironment = new Environment(this);
        EnvironmentMap mEnvironmentMap = new MyEnvironmentMap(this, mEnvironment);
        mEnvironment.map(mEnvironmentMap);
        */
        mEnvironment = new Environment(this, Environment.MAP_RANDOM_WALLS);

        mRobot = new MyRobot(mEnvironment);
        mEnvironment.add(mRobot);
        //mRobot.position(-400, -350);

        mTrace = new Trace(mRobot);
        mOSD = new OSD(this, mEnvironment);
    }

    class MyEnvironmentMap extends EnvironmentMapEditor {
        MyEnvironmentMap(PApplet pParent, Environment pEnvironment) {
            super(pParent, pEnvironment);

            pEnvironment.target().position().x = 300;
            pEnvironment.target().position().y = 220;
        }

        @Override
        public void fillCornersAndBalls() {
            addCorner(0.11328125f, 0.1953125f);
            addCorner(0.11328125f, 0.203125f);
            addCorner(0.021484375f, 0.54296875f);
            addCorner(0.037109375f, 0.8671875f);
            addCorner(0.037109375f, 0.9707031f);
            addCorner(0.140625f, 0.9863281f);
            addCorner(0.12109375f, 0.8925781f);
            addCorner(0.099609375f, 0.7675781f);
            addCorner(0.13085938f, 0.42578125f);
            addCorner(0.203125f, 0.24414062f);
            addCorner(0.20703125f, 0.21875f);
            addCorner(0.29101562f, 0.21679688f);
            addCorner(0.25585938f, 0.33007812f);
            addCorner(0.22851562f, 0.48242188f);
            addCorner(0.19726562f, 0.6972656f);
            addCorner(0.21289062f, 0.8203125f);
            addCorner(0.22851562f, 0.9199219f);
            addCorner(0.34765625f, 0.9511719f);
            addCorner(0.39648438f, 0.921875f);
            addCorner(0.35742188f, 0.890625f);
            addCorner(0.29492188f, 0.8417969f);
            addCorner(0.30664062f, 0.7832031f);
            addCorner(0.36914062f, 0.71484375f);
            addCorner(0.42578125f, 0.8066406f);
            addCorner(0.4921875f, 0.890625f);
            addCorner(0.5957031f, 0.921875f);
            addCorner(0.6503906f, 0.875f);
            addCorner(0.5488281f, 0.8105469f);
            addCorner(0.45703125f, 0.609375f);
            addCorner(0.3515625f, 0.6074219f);
            addCorner(0.27539062f, 0.6582031f);
            addCorner(0.33984375f, 0.4765625f);
            addCorner(0.35546875f, 0.34960938f);
            addCorner(0.48828125f, 0.47851562f);
            addCorner(0.48632812f, 0.4765625f);
            addCorner(0.5097656f, 0.50390625f);
            addCorner(0.6484375f, 0.5800781f);
            addCorner(0.8535156f, 0.7050781f);
            addCorner(0.859375f, 0.5097656f);
            addCorner(0.8105469f, 0.34179688f);
            addCorner(0.8613281f, 0.16015625f);
            addCorner(0.68359375f, 0.1796875f);
            addCorner(0.734375f, 0.36523438f);
            addCorner(0.7988281f, 0.5234375f);
            addCorner(0.7324219f, 0.5488281f);
            addCorner(0.6347656f, 0.5019531f);
            addCorner(0.5410156f, 0.38671875f);
            addCorner(0.44335938f, 0.33789062f);
            addCorner(0.40625f, 0.26367188f);
            addCorner(0.39648438f, 0.16992188f);
            addCorner(0.5859375f, 0.27539062f);
            addCorner(0.640625f, 0.38085938f);
            addCorner(0.6875f, 0.34765625f);
            addCorner(0.6484375f, 0.2578125f);
            addCorner(0.58203125f, 0.13085938f);
            addCorner(0.7207031f, 0.08984375f);
            addCorner(0.6269531f, 0.041015625f);
            addCorner(0.25390625f, 0.060546875f);
            addCorner(0.025390625f, 0.04296875f);
            addCorner(0.12890625f, 0.1015625f);
            addCorner(0.2890625f, 0.103515625f);
            addCorner(0.33789062f, 0.14257812f);
            addCorner(0.25390625f, 0.15820312f);
            addCorner(0.16601562f, 0.15625f);
            addCorner(0.11328125f, 0.19921875f);
            addBall(0.15039062f, 0.21875f);
            addBall(0.30664062f, 0.19726562f);
            addBall(0.39648438f, 0.111328125f);
            addBall(0.484375f, 0.14648438f);
            addBall(0.50390625f, 0.10546875f);
            addBall(0.7578125f, 0.25f);
            addBall(0.76953125f, 0.31835938f);
            addBall(0.81640625f, 0.6191406f);
            addBall(0.5019531f, 0.4296875f);
            addBall(0.5488281f, 0.4609375f);
            addBall(0.53515625f, 0.4296875f);
            addBall(0.35351562f, 0.29492188f);
            addBall(0.30273438f, 0.34765625f);
            addBall(0.29101562f, 0.46484375f);
            addBall(0.2734375f, 0.41796875f);
            addBall(0.3125f, 0.41210938f);
            addBall(0.23828125f, 0.66015625f);
            addBall(0.26953125f, 0.7421875f);
            addBall(0.3984375f, 0.6777344f);
            addBall(0.47265625f, 0.7890625f);
            addBall(0.0703125f, 0.8691406f);
            addBall(0.080078125f, 0.51171875f);
            addBall(0.13085938f, 0.30664062f);
        }
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

        /**
         * Knoten eines Rasters. Wird für A* verwendet.
         */
        class Quad {

            private float mapX;
            private float mapY;

            /**
             * Speichert zusätzlich einen Betrag an Aufwand, der auf die Heuristik in A* addiert wird.
             */
            private float effort;

            /**
             * Heuristic für A*. Speichert die Luftlinie.
             */
            private float h;

            /**
             * f(x) = distToStart + h
             */
            private float f;

            /**
             * Gibt an ob der Node ein Obstacle ist.
             */
            private boolean obstacale;

            /**
             * Bestimmt das verhalten. -1 ist Grundwert.
             */
            private int obstacleTyp;

            public int getObstacleTyp() {
                return obstacleTyp;
            }

            public void setObstacleTyp(int obstacleTyp) {
                this.obstacleTyp = obstacleTyp;
            }

            /**
             * Vorgängerknoten.
             */
            private Quad predecessor;

            /**
             * Distanz zum Startpunkt.
             */
            private float distToStart;

            /**
             * Farbwert eines Nodes
             */
            private float[] color = new float[3];

            /**
             * Zähler für den Knoten, ob es der erste … n-te Knoten ist.
             */
            private int id;

            /**
             * Erzeugt einen neuen Node.
             * @param h Luftlinie + Gefahrenzuschlag
             * @param obstacale Hinderniss ja/nein
             */
            public Quad(final float h, final boolean obstacale, final float mapX,final float mapY) {
                if (h < 0 || mapX < 0 || mapY < 0) throw new IllegalArgumentException();
                this.h = h;
                this.mapX = mapX;
                this.mapY = mapY;
                this.obstacale = obstacale;
                predecessor = null;
                distToStart = 0;
                //Nichts ist -1
                obstacleTyp = -1;
                id = 0;
                f = Float.MAX_VALUE;
                effort = 0;
                //Lege Farbe fest
                color[0] = 0;
                color[1] = 0;
                color[2] = 0;
            }

            public int getId() {
                return id;
            }

            public void setId(int id) {
                this.id = id;
            }

            public float getMapX() {
                return mapX;
            }

            public void setMapX(float mapX) {
                this.mapX = mapX;
            }

            public float getMapY() {
                return mapY;
            }

            public void setMapY(float mapY) {
                this.mapY = mapY;
            }

            public float getF() {
                return f;
            }

            public void setF(float f) {
                if (f < 0) throw new IllegalArgumentException();
                this.f = f;
            }

            public float getH() {
                return h;
            }

            public void setH(final float h) {
                if (h < 0) throw new IllegalArgumentException();
                this.h = h;
            }

            public boolean isObstacale() {
                return obstacale;
            }

            public void setObstacale(final boolean obstacale) {
                this.obstacale = obstacale;
            }

            public Quad getPredecessor() {
                return predecessor;
            }

            public void setPredecessor(final Quad predecessor) {
                this.predecessor = predecessor;
            }

            public float getDistToStart() {
                return distToStart;
            }

            public void setDistToStart(final float distToStart) {
                if (distToStart < 0) throw new IllegalArgumentException();
                this.distToStart = distToStart;
            }

            /**
             * Gibt Array zurück, der verändert werden darf.
             * @return Array zum verändern der Farbwerte.
             */
            public float[] getColor() {
                return color;
            }

            public float getEffort() {
                return effort;
            }

            public void setEffort(float effort) {
                this.effort = effort;
            }

            @Override
            public String toString() {
                return "Quad{" +
                        "f=" + f +
                        '}';
            }
        }

        /**
         * Gibt die größe eines Rasterfelds an. Gibt auch implizit an, welche abstände zwischen den Knoten existieren.
         */
        private final float QUAD_SIZE_MIN = 1;
        /**
         * Diagonale Länge
         */
        private final float QUAD_SIZE_MAX = sqrt(QUAD_SIZE_MIN*QUAD_SIZE_MIN + QUAD_SIZE_MIN*QUAD_SIZE_MIN);

        /**
         * Breite der A* Map. Muss durch 2 ohne Rest teilbar sein.
         */
        private final int MAP_WIDTH = (int) (1024/ QUAD_SIZE_MIN);
        /**
         * Höhe der A* Map. Muss durch 2 ohne Rest teilbar sein.
         */
        private final int MAP_HEIGHT = (int) (1024/ QUAD_SIZE_MIN);

        /**
         * Raster für A*
         */
        private final Quad[][] map;

        /**
         * Enthält die Nodes, welche noch nicht bearbeitet wurden.
         * ColsedList (Liste der bearbeiteten Nodes) wird indirekt implementiert.
         */
        private final PriorityQueue<Quad> openList;
        /**
         * Ziel Quadrant.
         */
        private Quad targetQuad;

        /**
         * Fahrziel
         */
        private Quad wayPoint;

        /**
         * Id des WayPoints
         */
        private final int MAX_WAY_POINT_ID = 24;

        /**
         * Ziel als Vec2
         */
        private final Vec2 targetPosition;

        private final float RADIUS = 20;
        private final float DEBUG_SCREEN = 80;

        private final float RED = 255;
        private final float GREEN = 255;

        /**
         * Wahl der Zahl: Florian
         */
        private final float EFFORT = 2.8f;

        private final float EFFORT_REDUCTION = 0.5f;

        /**
         * Comparator für PriorityQueue
         */
        private final class QuadComparator implements Comparator<Quad> {

            /**
             * Der Quadrant mit dem kleinsten f Wert hat Priorität.
             * Wenn {@code null} für eines der beiden Node-Objekte übergeben wird, fliegt eine {@link java.lang.IllegalArgumentException}
             *
             * @param quad1 Knoten, der mit Knoten quad2 verglichen werden soll.
             * @param quad2 Knoten, der mit Knoten quad1 verglichen werden soll.
             * @return f von quad1 == f von quad2 -> 0 wird zurückgegeben.
             * f von quad1 < f von quad2 -> ein Wert echt kleiner 0 wird zurückgegeben
             * f von quad1 > f von quad2 -> ein Wert echt größer 0 wird zurückgegeben
             *
             **/
            @Override
            public int compare(Quad quad1, Quad quad2) {
                if (quad1 == null || quad2 == null) throw new IllegalArgumentException();
                float result = quad1.getF() - quad2.getF();
                if (result < 0) {
                    return -1;
                } else if (result > 0) {
                    return 1;
                } else {
                    return 0;
                }
            }
        }

        /**
         * Container für nützliche Daten über den Roboter aus dem letzten Frame
         */
        class LastRobo {

            private float steer;

            public float getSteer() {
                return steer;
            }

            public Vec2 getPosition() {
                return lastPosition;
            }

            private Vec2 lastPosition;

            public LastRobo(final float steer, final Vec2 lastPosition) {
                if (lastPosition == null) {
                    throw new IllegalArgumentException();
                }
                this.steer = steer;
                this.lastPosition = new Vec2 (lastPosition);
            }
        }

        /**
         * RingBuffer für Roboter Daten. Hat feste Größe. NUR push verwenden!!!, wenn man hinzufügen möchte.
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
         * Speichert den Bewegungsstatus FORWARD, BACKWARD.
         * Andere Stati sind nicht zulässig.
         */
        private ROBO_STATUS status;

        private boolean statnding;

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
        private final RingBuffer<LastRobo> lastRobos;

        /**
         * Die letzen Wegpunkte, die nicht null sind.
         */
        private final ArrayDeque<Quad> wayPoints;

        /**
         * Epsilon für das isStanding-Prädikat
         */
        private final float EPS = 0.04f;

        MyRobot(Environment pEnvironment) {
            super(pEnvironment);

            sensors = new Sensor[5];
            lastRobos = new RingBuffer<>(LAST_ROBO_SIZE);
            map = new Quad[MAP_WIDTH][MAP_HEIGHT];
            openList = new PriorityQueue<>(MAP_WIDTH*MAP_HEIGHT, new QuadComparator());
            targetPosition = mEnvironment.target().position();
            fillMap();
            wayPoint = null;
            wayPoints = new ArrayDeque<>();
            timer = 0;

            sensors[0] = addSensor(-PI/6    , maxSensorRange);
            sensors[1] = addSensor(PI/6     , maxSensorRange);
            sensors[2] = addSensor(-PI/2.5f , maxSensorRange);
            sensors[3] = addSensor(PI/2.5f  , maxSensorRange);
            //tentacle
            sensors[4] = addSensor(0, maxSensorRange);


            direction = ROBO_STATUS.NO_STEERING;
            status = ROBO_STATUS.FORWARD;
            statnding = false;
        }

        /**
         * Füllt die Map und initialisiert die einzelnen Quadranten
         */
        private void fillMap() {
            Vec2 targetQuadVec = worldToQuadpoint(targetPosition);

            for (int width = 0; width < MAP_WIDTH; width ++) {
                for (int height = 0; height < MAP_HEIGHT; height++) {
                    map[width][height] = new Quad( targetQuadVec.sub(new Vec2(width, height)).length(), false, width, height);
                }
            }

            targetQuad = map[(int) targetQuadVec.x][(int) targetQuadVec.y];
            targetQuad.setH(0);
        }

        /**
         * Wandelt einen Punkt in map Koordinaten um, sprich den Quadrantenpunkt.
         *
         * @param point muss im Bereich zwischen width und height liegen
         * @return Quadrantenpunkt.
         */
        private Vec2 worldToQuadpoint(Vec2 point) {
            if (point == null) {
                throw new IllegalArgumentException();
            }
            float x = Math.round(point.x) + MAP_WIDTH/2;
            float y = (-1)*((point.y < 0 ? Math.round(point.y) : Math.round(point.y)) - MAP_HEIGHT/2);
            if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) {
                throw new IllegalArgumentException();
            }
            return new Vec2(x,y);
        }

        /**
         * Wandelt einen Punkt in MapKoordinaten in Weltkoordinaten.
         *
         * @param point
         * @return
         */
        private Vec2 quadToWorldpoint(Vec2 point) {
            if (point == null) {
                throw new IllegalArgumentException();
            }

            float x = point.x - MAP_WIDTH/2;
            float y = (-1)*point.y + MAP_HEIGHT/2;
            return new Vec2(x, y);
        }

        /**
         * Timer für Aktionen über Zeit
         */
        private float timer;

        public void update(float pDeltaTime) {

            mAngle += pDeltaTime * 20;

            switch (status) {
                case FORWARD:
                    sensors[0].angle(sin(mAngle * 0.3f) -PI/6);
                    sensors[1].angle(sin(-mAngle * 0.3f) + PI/6);
                    sensors[2].angle(sin(-mAngle * 0.3f) -PI/2.5f);
                    sensors[3].angle(sin(mAngle * 0.3f) + PI/2.5f);
                    sensors[4].angle(sin(mAngle) * 1/2 - steer());
                    break;
                case BACKWARD:
                    sensors[0].angle(sin(mAngle * 0.3f) -5*PI/6);
                    sensors[1].angle(sin(-mAngle * 0.3f) + 5*PI/6);
                    sensors[2].angle(sin(-mAngle * 0.3f) -PI/2f);
                    sensors[3].angle(sin(mAngle * 0.3f) + PI/2f);
                    sensors[4].angle(sin(mAngle) * 1/2 - steer() + PI);
                    break;
                default:
                    sensors[0].angle(sin(mAngle * 0.3f) -PI/6);
                    sensors[1].angle(sin(-mAngle * 0.3f) + PI/6);
                    sensors[2].angle(sin(-mAngle * 0.3f) -PI/2.5f);
                    sensors[3].angle(sin(mAngle * 0.3f) + PI/2.5f);
                    sensors[4].angle(sin(mAngle) * 1/2);
                    break;
            }

            for (Sensor sensor : sensors) {
                if (sensor.triggered()) {
                    addObstacle(sensor.obstacle(), Sensor.WALL);//sensor.obstacleType()); Weil Walls nicht walls sind … TODO
                }
            }

            Quad quad = aStar();
            int maxWaxPoints = generatePath(quad);
            int delta = Math.round(MAX_WAY_POINT_ID /
                    (speed() < 0 ? maxBackwardSpeed : maxForwardSpeed) * speed());
            int wayPointId = maxWaxPoints - delta;

            float angleToGoal = 0;
            if (!wayPoints.isEmpty()) {
                for (Quad q : wayPoints) {
                    if (q.getId() == wayPointId) {
                        wayPoint = q;
                        angleToGoal = angleToGoal(quadToWorldpoint(new Vec2(wayPoint.getMapX(), wayPoint.getMapY())));
                        steer(status == ROBO_STATUS.BACKWARD ? -angleToGoal : angleToGoal);
                        break;
                    }
                }
            }

            println(timer);
            println(status);
            if (timer <= 0) {
                if (angleToGoal < -PI/2f) {
                    timer = pDeltaTime*12;
                    status = ROBO_STATUS.BACKWARD;

                } else if (angleToGoal > PI / 2f) {
                    timer = pDeltaTime * 12;
                    status = ROBO_STATUS.BACKWARD;

                } else {
                    status = ROBO_STATUS.FORWARD;
                }
            }

            Sensor sensor;
            switch (status) {
                case FORWARD:
                    sensor = mostDisturbingFrontObstacle(sensors);

                    if (sensor != null) {
                        speed(maxForwardSpeed * sensor.obstacleDistance() * 1.8f);
                    } else {
                        speed(maxForwardSpeed);
                    }

                    break;
                case BACKWARD:
                    sensor = mostDisturbingBackObstacle(sensors);

                    if (sensor != null) {
                        speed(maxBackwardSpeed * sensor.obstacleDistance() * 1.8f);
                    } else {
                        speed(maxBackwardSpeed);
                    }
                    timer -= pDeltaTime;

                    break;
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
                        status = ROBO_STATUS.FORWARD;
                        break;
                    case 's':
                        speed(0);
                        steer(0);
                        break;
                    case  'x':
                        speed(maxBackwardSpeed);
                        status = ROBO_STATUS.BACKWARD;
                        break;
                }
            }

            //Save LastFrameRobo -> Before State Update !!!
            lastRobos.push(new LastRobo(steer(), position()));
            //Update States
            updateStates();

        }

        /**
         * Erzeugt ein Route.
         *
         * @param quad
         */
        private int generatePath(Quad quad) {
            wayPoints.clear();

            int id = 0;
            while (quad != null) {
                quad.setId(id++);
                wayPoints.push(quad);
                quad = quad.getPredecessor();
            }
            return id;
        }

        /**
         * Setzt Daten zurück, die von A* verändert wurden.
         */
        private void clearMap() {
            for (int i = 0; i < MAP_WIDTH; i++) {
                for (int j = 0; j < MAP_HEIGHT; j++) {
                    Quad quad = map[i][j];
                    quad.setF(MAX_FLOAT);
                    quad.setPredecessor(null);
                    quad.setDistToStart(0);
                    quad.setId(0);

                    switch (quad.getObstacleTyp()) {
                        case Sensor.BALL :
                            quad.setEffort(quad.getEffort() - EFFORT_REDUCTION * 0.6f);
                            if (quad.getEffort() < 0) {
                                quad.setEffort(0);
                                quad.setObstacleTyp(-1);
                            }
                            break;
                        case Sensor.UNKNOWN :
                            quad.setEffort(quad.getEffort() - EFFORT_REDUCTION * 0.9f);
                            if (quad.getEffort() < 0) {
                                quad.setEffort(0);
                                quad.setObstacleTyp(-1);
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        /**
         * Führt A* aus. Target muss erreichbar sein.
         *
         */
        public Quad aStar() {

            clearMap();
            openList.clear();

            Vec2 startPoint = worldToQuadpoint(position());
            Quad startQuad = map[(int) startPoint.x][(int) startPoint.y];
            startQuad.setF(0);
            openList.offer(startQuad);

            while(!openList.isEmpty()) {
                Quad quad = openList.poll();


                //Weg zum Ziel gefunden
                if (quad == targetQuad) {
                    return quad;
                }

                //println("noch in Arbeit");
                expandPath(quad);
            }
            //Wenn kein Ziel gefunden werden konnte
            return null;
        }

        /**
         * Erweitert die openList.
         * @param quad
         */
        private void expandPath(final Quad quad) {
            int mapX = (int) quad.getMapX();
            int mapY = (int) quad.getMapY();

            Quad successor;
            if (validQuad(mapX, ++mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MIN, quad);

            }
            if (validQuad(--mapX, mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MAX, quad);

            }
            if (validQuad(mapX, --mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MIN, quad);

            }
            if (validQuad(mapX, --mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MAX, quad);

            }
            if (validQuad(++mapX, mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MIN, quad);

            }
            if (validQuad(++mapX, mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MAX, quad);

            }
            if (validQuad(mapX, ++mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MIN, quad);

            }
            if (validQuad(mapX, ++mapY)) {
                successor = map[mapX][mapY];
                updateSucessor(successor, QUAD_SIZE_MAX, quad);
            }
        }

        /**
         * Checkt ob der Quad noch in der Map liegt.
         *
         * @param mapX
         * @param mapY
         * @return
         */
        private boolean validQuad(final int mapX, final int mapY) {
            if ((mapX < 0 || mapX >= MAP_WIDTH) || (mapY < 0 || mapY >= MAP_HEIGHT)) {
                return false;
            }
            return true;
        }

        /**
         * Update Successor. Nur hinzufügen, wenn der neue f Wert kleiner als der Alte ist.
         *
         * @param successor
         * @param costsInDist
         * @param possiblePredecessor
         */
        private void updateSucessor(final Quad successor, final float costsInDist, final Quad possiblePredecessor) {
            float tentativeDistToStart = successor.getDistToStart() + costsInDist;
            float f = tentativeDistToStart + successor.getH() + successor.getEffort();

            if ((!openList.contains(successor) || tentativeDistToStart < successor.getDistToStart())
                    && successor.getF() > f && !successor.isObstacale()) {
                successor.setPredecessor(possiblePredecessor);
                successor.setDistToStart(tentativeDistToStart);
                successor.setF(f);
                if (!openList.contains(successor)) {
                    openList.offer(successor);
                }
            }
        }

        /**
         * Fügt ein Obstacle in die Map ein. In einem Radius um ein Obstacle, wird die Heuristic der Quadranten verändert.
         *
         * @param opstaclePosition Position des Hindernisses
         */
        public void addObstacle(final Vec2 opstaclePosition, final int obstacleTyp) {
            if (opstaclePosition == null) throw new IllegalArgumentException();

            Vec2 obstacleMapPoint = worldToQuadpoint(opstaclePosition);
            Quad obstacle = map[(int) obstacleMapPoint.x][(int) obstacleMapPoint.y];

            int WEST  = (int) ((obstacleMapPoint.x - RADIUS) < 0 ? 0 : (obstacleMapPoint.x - RADIUS));
            int NORTH = (int) ((obstacleMapPoint.y - RADIUS) < 0 ? 0 : (obstacleMapPoint.y - RADIUS));
            int EAST  = (int) ((obstacleMapPoint.x + RADIUS) >= MAP_WIDTH  ? MAP_WIDTH  - 1 : (obstacleMapPoint.x + RADIUS));
            int SOUTH = (int) ((obstacleMapPoint.y + RADIUS) >= MAP_HEIGHT ? MAP_HEIGHT - 1 : (obstacleMapPoint.y + RADIUS));

            //Aufwandsberechnung für Hindernisse:
            //Wände werden NICHT durch andere Hindernisse überlagert
            //Unknown überlagert Ball

            switch (obstacleTyp) {
                case Sensor.UNKNOWN:
                    obstacle.setObstacleTyp(Sensor.UNKNOWN);
                    obstacle.setEffort(RADIUS*EFFORT);

                    for (int i = WEST; i <= EAST; i++) {
                        for (int j = NORTH; j <= SOUTH; j++) {
                            float lenghtToOrigin = new Vec2(i, j).sub(obstacleMapPoint).length();
                            Quad quad = map[i][j];
                            if (lenghtToOrigin <= RADIUS && quad.getObstacleTyp() != Sensor.WALL) {
                                //Felder werden makiert
                                quad.setObstacleTyp(Sensor.UNKNOWN);

                                float newEffort = (RADIUS-lenghtToOrigin) * EFFORT;
                                if (newEffort > quad.getEffort()) {
                                    quad.setEffort(newEffort);
                                }
                            }
                        }
                    }
                    break;
                case Sensor.WALL:
                    obstacle.setObstacleTyp(Sensor.WALL);
                    obstacle.setEffort(RADIUS*EFFORT);
                    obstacle.setObstacale(true);

                    for (int i = WEST; i <= EAST; i++) {
                        for (int j = NORTH; j <= SOUTH; j++) {
                            float lenghtToOrigin = new Vec2(i, j).sub(obstacleMapPoint).length();
                            Quad quad = map[i][j];
                            if (lenghtToOrigin <= RADIUS) {
                                //Felder werden makiert
                                quad.setObstacleTyp(Sensor.WALL);

                                float newEffort = (RADIUS-lenghtToOrigin) * EFFORT;

                                switch (quad.getObstacleTyp()) {
                                    case Sensor.UNKNOWN:
                                        quad.setEffort(newEffort);
                                        break;
                                    case Sensor.BALL:
                                        quad.setEffort(newEffort);
                                        break;
                                    default:
                                        if (newEffort > quad.getEffort()) {
                                            quad.setEffort(newEffort);
                                        }
                                        break;
                                }
                            }
                        }
                    }
                    break;
                case Sensor.BALL:
                    obstacle.setObstacleTyp(Sensor.BALL);
                    obstacle.setEffort(RADIUS*EFFORT);

                    for (int i = WEST; i <= EAST; i++) {
                        for (int j = NORTH; j <= SOUTH; j++) {
                            float lenghtToOrigin = new Vec2(i, j).sub(obstacleMapPoint).length();
                            Quad quad = map[i][j];
                            if (lenghtToOrigin <= RADIUS && quad.getObstacleTyp() != Sensor.WALL && quad.getObstacleTyp() != Sensor.UNKNOWN) {
                                //Felder werden makiert
                                quad.setObstacleTyp(Sensor.BALL);

                                float newEffort = (RADIUS-lenghtToOrigin) * EFFORT;
                                if (newEffort > quad.getEffort()) {
                                    quad.setEffort(newEffort);
                                }
                            }
                        }
                    }
                    break;
            }
        }

        /**
         * Zeichnet alle Obstacles ein und ihre Feldmanipulation.
         */
        public void drawMap() {
            Vec2 roboMapPoint = worldToQuadpoint(position());

            int WEST  = (int) ((roboMapPoint.x - DEBUG_SCREEN) < 0 ? 0 : (roboMapPoint.x - DEBUG_SCREEN));
            int NORTH = (int) ((roboMapPoint.y - DEBUG_SCREEN) < 0 ? 0 : (roboMapPoint.y - DEBUG_SCREEN));
            int EAST  = (int) ((roboMapPoint.x + DEBUG_SCREEN) >= MAP_WIDTH  ? MAP_WIDTH  - 1 : (roboMapPoint.x + DEBUG_SCREEN));
            int SOUTH = (int) ((roboMapPoint.y + DEBUG_SCREEN) >= MAP_HEIGHT ? MAP_HEIGHT - 1 : (roboMapPoint.y + DEBUG_SCREEN));

            noStroke();
            for (int i = WEST; i < EAST; i++) {
                for (int j = NORTH; j < SOUTH; j++) {
                    Quad quad = map[i][j];
                    if (quad.getEffort() > 0) {
                        quad.getColor()[0] = RED/(RADIUS * EFFORT) *quad.getEffort();
                        quad.getColor()[1] = GREEN - RED/(RADIUS * EFFORT) *quad.getEffort();
                        fill(quad.getColor()[0], quad.getColor()[1], quad.getColor()[2]);
                        Vec2 positionInWorld = quadToWorldpoint(new Vec2(i, j));
                        rect(positionInWorld.x, positionInWorld.y, QUAD_SIZE_MIN, QUAD_SIZE_MIN);
                    }
                    if (quad.isObstacale()) {
                        drawQuad(quad, 0, 0, 0);
                    }
                }
            }
        }

        /**
         * Zeichent ein spezielles Quad.
         *
         * Werte müssen valide sein.
         * @param quad
         * @param r
         * @param g
         * @param b
         */
        public void drawQuad(final Quad quad, final float r, final float g, final float b){
            fill(r, g, b);
            Vec2 positionInWorld = quadToWorldpoint(new Vec2(quad.getMapX(), quad.getMapY()));
            rect(positionInWorld.x, positionInWorld.y, QUAD_SIZE_MIN, QUAD_SIZE_MIN);
        }

        /**
         * Zeichnet Strecke von A* beginnt mit dem Ziel
         * @param pathStart
         */
        public void draw_A_STAR(final Quad pathStart) {
            noStroke();
            Quad quad = pathStart;
            while(quad != null) {
                drawQuad(quad, 0, 0, 255);
                quad = quad.getPredecessor();
            }
        }

        /**
         * Upadate der einzelnen States.
         */
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

            for (LastRobo robo : lastRobos) {
                if (robo.getSteer() <= 0) {
                    countLeftDirection++;
                } else {
                    countRightDirection++;
                }
            }

            //Is Stending nach Statustest
            if (-EPS < middleVelocity && middleVelocity < EPS) {
                statnding = true;
            } else {
                statnding = false;
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

        /**
         * returns the angle to the target. if the target is not defined this method
         * always returns 0.
         *
         * Für einen beliebigen Punkt.
         */
        public float angleToGoal(final Vec2 goal) {
            if (goal != null) {
                Vec2 target = goal;
                Vec2 pos = position();
                Vec2 relTarget = target.sub(pos);
                float relAngle = (float) Math.atan2(relTarget.y, relTarget.x);
                float ownAngle = clamp(body().getAngle());
                float goalAngle = clamp(relAngle - ownAngle - PApplet.PI / 2);
                return goalAngle;
            }
            return 0;
        }

        private float clamp(float goalAngle) {
            while (goalAngle > Math.PI) {
                goalAngle -= 2 * Math.PI;
            }
            while (goalAngle < -Math.PI) {
                goalAngle += 2 * Math.PI;
            }
            return goalAngle;
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

        private Sensor nearestObstacle(final Sensor[] sensors) {
            if (sensors == null) throw  new IllegalArgumentException();

            Sensor mostImportantSensor = null;
            float distanceToObstacle = MAX_FLOAT;

            for (Sensor s : sensors) {
                if (s.triggered()) {

                    if (s.obstacleDistance() < distanceToObstacle) {
                        distanceToObstacle = s.obstacleDistance();
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

            if (!statnding) {
                if (status == ROBO_STATUS.FORWARD) {
                    triangle(2, 2, 0, 4, -2, 2);
                } else if (status == ROBO_STATUS.BACKWARD) {
                    triangle(2, -2, 0, -4, -2, -2);
                }
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
            Vec2 target = targetPosition;
            line(target.x, target.y, position().x, position().y);

            drawMap();
            draw_A_STAR(targetQuad);
            if (wayPoint != null) {
                drawQuad(wayPoint, 200, 0, 100);
            }
        }
    }

    public static void main(String[] args) {
        PApplet.main(new String[]{playground.class.getName()});
    }
}
