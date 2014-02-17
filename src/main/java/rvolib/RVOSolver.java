package rvolib;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map.Entry;

import org.jgrapht.alg.DijkstraShortestPaths;
import org.jgrapht.alg.VisibilityGraphPlanner;
import org.jgrapht.util.Goal;

import rrt.JointWaypointState;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.discretization.LazyGrid;
import tt.euclid2i.region.Polygon;
import tt.euclid2i.region.Rectangle;
import tt.euclid2i.util.Util;
import tt.jointtraj.solver.SearchResult;

public class RVOSolver {

    /** Somewhere near goal the agent must adjust the preferred velocity to avoid overshooting **/
    private float adjustPrefferedVolcityNearGoalEpsilon = 0;
    /**	The agent is considered at goal if the distance between its current position and the goal is within this tolerance **/
    private float goalReachedToleranceEpsilon = 0; // 0.1f;

    private Point[] goals;
    /* maximum radius of Dijkstra expansion radius*/
    private float graphRadius;

    private outerLoopControl preferredVelocityControlerMethod = outerLoopControl.VISIBILITY_GRAPH;
    private Collection<Region> obstacles;
    private Collection<Region> moreInflatedObstacles;
    private Collection<Region> lessInflatedObstacles;
    private int iteration;
    private float jointCost = 0;
    private boolean showProgress;

    Simulator simulator;
    private Point[] starts;
    public RVOSolver(Point[] starts, Point[] goals, int bodyRadius,
                     Collection<Region> obstacles,
                     float timeStep, float neighborDist,
                     int maxNeighbors, float deconflictionTimeHorizonAgents,
                     float deconflictionTimeHorizonObstacles,
                     float maxSpeed, boolean showProgress) {

        this.showProgress = showProgress;
        this.simulator = new Simulator();
        this.simulator.setShowVis(showProgress);

        simulator.setTimeStep(timeStep);

        Vector2 initialVelocity = new Vector2();
        simulator.setAgentDefaults(neighborDist, maxNeighbors, deconflictionTimeHorizonAgents, deconflictionTimeHorizonObstacles,
                bodyRadius, maxSpeed, initialVelocity);


        // Add obstacles
        for (Region region : obstacles) {
            if (!(region instanceof Polygon)) {
                throw new RuntimeException("Only polygons are supported");
            }
            ArrayList<Vector2> obstacle = regionToVectorList((Polygon) region);
            simulator.addObstacle(obstacle);
        }

        simulator.processObstacles();

        this.starts = starts;
        this.goals = goals;

        // Add agents
        for (int i = 0; i < starts.length; i++) {
            Vector2 start = new Vector2(starts[i]);
            simulator.addAgent(start);
            simulator.getAgent(i).goal_ = goals[i];
        }

        this.graphRadius = 1000; // 2 * conflictRadius + 50;
        // VisManager.registerLayer(SimulationControlLayer.create(this));
        this.obstacles = obstacles;
        this.lessInflatedObstacles = Util.inflateRegions(obstacles, bodyRadius-1);
        this.moreInflatedObstacles = Util.inflateRegions(obstacles, bodyRadius);

        //this.goalReachedToleranceEpsilon = (float) (Math.ceil((double) bodyRadius / 5));
        //this.adjustPrefferedVolcityNearGoalEpsilon = (float) (Math.ceil((double) bodyRadius / 5));
    }

    public SearchResult solve(int iterationLimit, long interruptAtNs, double maxJointCost) {

        tt.euclid2i.EvaluatedTrajectory[] trajs = new tt.euclid2i.EvaluatedTrajectory[simulator.getNumAgents()];

//        for (Point goal : goals) {
//            for (Region r : obstacles) {
//                if (r.isInside(goal)) {
//                    //System.out.println("Goal point lies in an obstacle!");
//                    //return new SearchResult(trajs, false);
//                }
//            }
//        }

        switch (preferredVelocityControlerMethod) {
            case LAZY_GRID:
                createGridGraphController();
                break;
            case VISIBILITY_GRAPH:
                createVisibilityGraphController();
                break;
        }

        for (int i = 0; i < simulator.getNumAgents(); i++) {
            simulator.getAgent(i).clearTrajectory();
            simulator.getAgent(i).getCloseList().clear();
        }

        iteration = 0;
        do {
            switch (preferredVelocityControlerMethod) {
                case LAZY_GRID:
                    setLazyGridPreferredVelocities();
                    break;
                case VISIBILITY_GRAPH:
                    setVisibilityGraphPrefferedVelocities();
                    break;
            }
            simulator.doStep();

            // HARDCODED SIMULATION SPEED

            if (showProgress) {
                try {
                    Thread.sleep((long) 10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            iteration++;

            calculateJointCost();

            if (System.nanoTime() > interruptAtNs) {
                return new SearchResult(null, false);
            }

        } while (!reachedGoal() && iteration < iterationLimit && jointCost <= maxJointCost);

        for (int i = 0; i < trajs.length; i++) {
            trajs[i] = simulator.getAgent(i).getEvaluatedTrajectory(simulator.timeStep, goals[i], (int) Math.ceil(iterationLimit * simulator.timeStep));
        }
        if (iteration < iterationLimit && jointCost <= maxJointCost) {
            // System.out.println("RVO SUCCESS");
            return new SearchResult(trajs, true);
        } else {
            // System.out.println("RVO FAILURE");
            return new SearchResult(null, false);
        }
    }

    private void calculateJointCost() {
        for (int i = 0; i < simulator.getNumAgents(); ++i) {
            Point p = simulator
                    .getAgentPosition(i).convertToPoint2i();
            Point g = goals[i];
            if (!p.equals(g)) {
                jointCost += simulator.getTimeStep();
            }
        }
    }

    private void createVisibilityGraphController() {

        for (int i = 0; i < simulator.getNumAgents(); i++) {

            VisibilityGraphPlanner visibilityGraphPlanner = new VisibilityGraphPlanner(starts[i], goals[i], lessInflatedObstacles, moreInflatedObstacles, true);
            HashMap<Point, Double> nodeValueMap = visibilityGraphPlanner.evaluateGraph(10 * graphRadius, goals[i]);
            simulator.getAgent(i).setEvaluatedGraph(
                        nodeValueMap
                    );
        }
    }

    private void createGridGraphController() {
        for (int i = 0; i < simulator.getNumAgents(); ++i) {
            LazyGrid grid = new LazyGrid(goals[i],
                    obstacles, new Rectangle(new Point(-1000, -1000),
                    new Point(1000, 1000)),
                    LazyGrid.PATTERN_4_WAY_WAIT, 1/*
                                                 * 2 *
                                                 * (int)Simulator.getInstance
                                                 * ().defaultAgent_.radius_
                                                 */);
            simulator.getAgent(i).setGraph(grid);
            System.out.println("agent" + i + " grid created, goal: "
                    + goals[i]);
            DijkstraShortestPaths<Point, Line> dijkstra = new DijkstraShortestPaths<Point, Line>(
                    grid, goals[i], new Goal<Point>() {
                @Override
                public boolean isGoal(Point current) {
                    return false;
                }
            }, graphRadius);

            HashMap<Point, Double> lengths = dijkstra
                    .findLengths(100000000000l);
            simulator.getAgent(i).setEvaluatedGraph(lengths);
            System.out.println("agent" + i + " grid EVALUATED");
        }
    }

    private void setLazyGridPreferredVelocities() {
        for (int i = 0; i < simulator.getNumAgents(); ++i) {
            Vector2 p = simulator.getAgentPosition(i);

            if (p.convertToPoint2i().distance(goals[i]) < adjustPrefferedVolcityNearGoalEpsilon) {
                simulator.setAgentPrefVelocity(i,
                        Vector2.minus(new Vector2(goals[i]), p));
            } else {
                // c..center, u..up, d..down
                ArrayList<Point> neighbourPoints = new ArrayList<Point>();

                neighbourPoints.add(new Point((int) Math.floor(p.x()),
                        (int) Math.ceil(p.y()))); // cu
                neighbourPoints.add(new Point((int) Math.ceil(p.x()),
                        (int) Math.floor(p.y()))); // uc
                neighbourPoints.add(new Point((int) Math.ceil(p.x()),
                        (int) Math.ceil(p.y()))); // uu
                neighbourPoints.add(new Point((int) Math.floor(p.x()),
                        (int) Math.floor(p.y()))); // uu

                Point center = getBestPrefferedPoint(i, neighbourPoints);
                if (!center.equals(goals[i])) {
                    neighbourPoints = new ArrayList<Point>();
                    neighbourPoints.add(new Point(center.x - 1, center.y - 1)); // dd
                    neighbourPoints.add(new Point(center.x - 1, center.y)); // dc
                    neighbourPoints.add(new Point(center.x - 1, center.y + 1)); // du
                    neighbourPoints.add(new Point(center.x, center.y - 1)); // cd
                    neighbourPoints.add(new Point(center.x, center.y + 1)); // cu
                    neighbourPoints.add(new Point(center.x + 1, center.y - 1)); // ud
                    neighbourPoints.add(new Point(center.x + 1, center.y)); // uc
                    neighbourPoints.add(new Point(center.x + 1, center.y + 1)); // uu

                    center = getBestPrefferedPoint(i, neighbourPoints);
                }

                Vector2 result = new Vector2(center);

                Vector2 graphPointVector = Vector2.minus(result, simulator.getAgentPosition(i));
                // if (RVOMath.absSq(graphPointVector) > 1.0f) {
                graphPointVector = RVOMath.normalize(graphPointVector);
                // }
                simulator.setAgentPrefVelocity(i,
                        graphPointVector);
            }
        }
    }

    private void setVisibilityGraphPrefferedVelocities() {
        for (int i = 0; i < simulator.getNumAgents(); i++) {

            Vector2 currentPosition = simulator.getAgentPosition(i);
            double distanceToGoal = currentPosition.convertToPoint2i().distance(goals[i]);


            if (currentPosition.convertToPoint2i().distance(goals[i]) < adjustPrefferedVolcityNearGoalEpsilon) {
                simulator.setAgentPrefVelocity(i, new Vector2(0, 0));
            } else {

                HashMap<Point, Double> evaluatedGraph = simulator.getAgent(i).getEvaluatedGraph();

                double minTotalDistanceToGoal = Double.MAX_VALUE;
                Vector2 optimalPathDirection = null;

                for (Entry<Point, Double> pointValuePair : evaluatedGraph.entrySet()) {
                    Point point = pointValuePair.getKey();
                    double value =  pointValuePair.getValue();

                    Vector2 vector = new Vector2(point.x - currentPosition.x(),
                                                 point.y - currentPosition.y());

                    double distToCurrentPosition = vector.getLength();
                    double sumTotalDistToGoal = distToCurrentPosition + value;

                    double navigationEps = 0.1;
                    if (distToCurrentPosition < navigationEps  && !simulator.getAgent(i).getCloseList().contains(point)) {
                        simulator.getAgent(i).getCloseList().add(point);
                    }

                    if (sumTotalDistToGoal < minTotalDistanceToGoal && !simulator.getAgent(i).getCloseList().contains(point)) {
                        boolean obstacleConflict = false;

                        for (Region region : lessInflatedObstacles) {
                            if (region.intersectsLine(currentPosition.convertToPoint2i(), point)) {
                                obstacleConflict = true;
                                continue;
                            }
                        }
                        if (!obstacleConflict) {
                            optimalPathDirection = vector;
                            minTotalDistanceToGoal = sumTotalDistToGoal;
                        }
                    }
                }

                if (optimalPathDirection != null) {
                    optimalPathDirection = RVOMath.normalize(optimalPathDirection);
                    float maxSpeed = simulator.getAgentMaxSpeed();

                    if (distanceToGoal > simulator.timeStep * maxSpeed) {
                        // it will take some time to reach the goal
                        optimalPathDirection = Vector2.scale(maxSpeed, optimalPathDirection);
                    } else {
                        // goal will be reached in the next time step
                        double speed = distanceToGoal/maxSpeed;
                        optimalPathDirection = Vector2.scale((float)speed, optimalPathDirection);
                    }

                    simulator.setAgentPrefVelocity(i, optimalPathDirection);
                } else {
                    // Cannot find optimal path to goal for some reason
                    Vector2 toGoalVelocityVector = new Vector2(
                            goals[i].x - currentPosition.x_, goals[i].y - currentPosition.y_);
                    float maxSpeed = simulator.getAgentMaxSpeed();
                    optimalPathDirection = Vector2.scale(maxSpeed, toGoalVelocityVector);
                    simulator.setAgentPrefVelocity(i, optimalPathDirection);
                    System.err.println("No path to goal found! Using straight direction to goal...");
                }
            }
        }
    }

    private Point getBestPrefferedPoint(int i, ArrayList<Point> neighbourPoints) {
        HashMap<Point, Double> evaluatedGraph = simulator
                .getAgent(i).getEvaluatedGraph();
        Double[] values = new Double[neighbourPoints.size()];
        Point p = simulator.getAgentPosition(i)
                .convertToPoint2i();

        for (int j = 0; j < neighbourPoints.size(); j++) {
            boolean obstacleConflict = false;
            for (Region region : obstacles) {
                if (region.intersectsLine(p, neighbourPoints.get(j))) {
                    obstacleConflict = true;
                    continue;
                }
            }
            if (!obstacleConflict) {
                values[j] = evaluatedGraph.get(neighbourPoints.get(j));
            } else {
                values[j] = null;
            }
        }

        // TODO add distance to the point
        // find min
        double min = Double.MAX_VALUE;
        int index = -1;
        for (int j = 0; j < neighbourPoints.size(); j++) {
            if (values[j] != null && values[j] < min) {
                min = values[j];
                index = j;
            }
        }
        // if all values are null, the agent is too far from the generated graph
        if (index == -1) {
            return goals[i];
        }
        return (neighbourPoints.get(index));
    }

    private boolean reachedGoal() {
        /* Check if all agents have reached their goals. */
        float eps = 1f;
        for (int i = 0; i < simulator.getNumAgents(); ++i) {
            Point p = simulator
                    .getAgentPosition(i).convertToPoint2i();
            Point g = goals[i];

            if (p.distance(g) > goalReachedToleranceEpsilon) {
                return false;
            }
        }

        return true;
    }

    public static enum outerLoopControl {
        LAZY_GRID,
        VISIBILITY_GRAPH
    }

    /**
     * called from RRT algorithm
     *
     * @param from
     * @param to
     */
    public void setupProblem(JointWaypointState from, JointWaypointState to) {
        goals = new Point[from.nAgents()];
        for (int i = 0; i < simulator.getNumAgents(); i++) {
            simulator.getAgent(i).position_ = new Vector2(
                    from.getPosition(i));
            simulator.getAgent(i).goal_ = to.getPosition(i);
            goals[i] = to.getPosition(i);
        }
    }

    public int getIterations() {
        return iteration;
    }

    private ArrayList<Vector2> regionToVectorList(Polygon polygon) {

        Point[] points = polygon.getPoints();

        ArrayList<Vector2> obstacle = new ArrayList<Vector2>();

        for (Point point : points) {
            obstacle.add(new Vector2(point.x, point.y));
        }
        return obstacle;
    }
}
