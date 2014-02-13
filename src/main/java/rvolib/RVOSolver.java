package rvolib;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

import org.jgrapht.WeightedGraph;
import org.jgrapht.alg.DijkstraShortestPaths;
import org.jgrapht.alg.VisibilityGraphPlanner;
import org.jgrapht.util.Goal;

import rrt.JointWaypointState;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.discretization.LazyGrid;
import tt.euclid2i.discretization.VisibilityGraph;
import tt.euclid2i.region.Rectangle;
import tt.euclid2i.util.Util;
import tt.jointtraj.solver.SearchResult;

public class RVOSolver {

    /** Somewhere near goal the agent must adjust the preferred velocity to avoid overshooting **/
    private float adjustPrefferedVolcityNearGoalEpsilon = 0;
    /**	The agent is considered at goal if the distance between its current position and the goal is within this tolerance **/
    private float goalReachedToleranceEpsilon = 0; // 0.1f;

    private Point[] goals;
    private float graphRadius;

    private outerLoopControl preferredVelocityControlerMethod = outerLoopControl.VISIBILITY_GRAPH;
    private VisibilityGraphPlanner visibilityGraphPlanner;
    private Collection<Region> obstacles;
    private Collection<Region> moreInflatedObstacles;
    private Collection<Region> lessInflatedObstacles;
    private int iteration;
    private float jointCost = 0;
    private boolean showProgress;

    Simulator simulator;
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
            // We support only rectangles?
            if (!(region instanceof Rectangle)) {
                throw new RuntimeException("Only rectangles are supported");
                //region = region.getBoundingBox();
            }
            ArrayList<Vector2> obstacle = regionToVectorList((Rectangle) region);
            simulator.addObstacle(obstacle);
        }

        simulator.processObstacles();

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

        for (Point goal : goals) {
            for (Region r : obstacles) {
                if (r.isInside(goal)) {
                    // System.out.println("RVO FAILURE");
                    return new SearchResult(trajs, false);
                }
            }
        }

        switch (preferredVelocityControlerMethod) {
            case LAZY_GRID:
                createGridGraphController();
                break;
            case VISIBILITY_GRAPH:
                createVisibilityGraphController();
                break;
        }

        for (int i = 0; i < simulator.getNumAgents(); ++i) {
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
//			System.out.println(iteration);

            calculateJointCost();
//			System.out.println("threshold: " + maxJointCost + " optimal solution cost: " + OptimalSolutionProvider.getInstance().getOptimalSolutionCost() + " cost: " + jointCost);

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
        WeightedGraph<Point, Line> visibilityGraphAroundObstacles = VisibilityGraph.createVisibilityGraph(lessInflatedObstacles, moreInflatedObstacles, Collections.EMPTY_LIST);
        for (int i = 0; i < simulator.getNumAgents(); i++) {
            visibilityGraphPlanner = new VisibilityGraphPlanner(visibilityGraphAroundObstacles, lessInflatedObstacles, true);
            visibilityGraphPlanner.createVisibilityGraph(goals[i]);
            simulator.getAgent(i).setEvaluatedGraph(
                    visibilityGraphPlanner.evaluateGraph(i, 10 * graphRadius,
                    goals[i]));
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
        for (int i = 0; i < simulator.getNumAgents(); ++i) {

            Vector2 currentPosition = simulator.getAgentPosition(i);
            double distanceToGoal = currentPosition.convertToPoint2i().distance(goals[i]);


            if (currentPosition.convertToPoint2i().distance(goals[i]) < adjustPrefferedVolcityNearGoalEpsilon) {
                simulator.setAgentPrefVelocity(i, new Vector2(0, 0));
            } else {
                HashMap<Point, Double> evaluatedGraph = simulator
                        .getAgent(i).getEvaluatedGraph();

                Iterator<Entry<Point, Double>> it = evaluatedGraph.entrySet()
                        .iterator();
                double minDistance = Double.MAX_VALUE;
                Vector2 result = null;
                while (it.hasNext()) {
                    Map.Entry pairs = (Map.Entry) it.next();
                    double value = (Double) pairs.getValue();
                    Point point = (Point) pairs.getKey();

                    Vector2 vector = new Vector2(point.x - currentPosition.x(), point.y
                            - currentPosition.y());
                    double dist = vector.getLength();
                    double sumDistance = dist + value;
                    double navigationEps = 0.1;

                    if (dist < navigationEps
                            && !simulator.getAgent(i)
                            .getCloseList().contains(point)) {
                        simulator.getAgent(i).getCloseList()
                                .add(point);

                    }

                    if (sumDistance < minDistance
                            && !simulator.getAgent(i)
                            .getCloseList().contains(point)) {
                        boolean obstacleConflict = false;
                        Point pos = currentPosition.convertToPoint2i();
                        double eps = Double.MAX_VALUE;
                        // optimalization for maze
                        // double eps = 250;

                        for (Region region : lessInflatedObstacles) {
                            if (region.intersectsLine(currentPosition.convertToPoint2i(),
                                    point)) {
                                obstacleConflict = true;
                                continue;
                            }
                        }
                        if (!obstacleConflict) {
                            result = vector;
                            minDistance = sumDistance;
                        }
                    }
                }

                if (result != null) {
                    result = RVOMath.normalize(result);
                    float maxSpeed = simulator.getAgentMaxSpeed();

                    if (distanceToGoal > simulator.timeStep * maxSpeed) {
                        // it will take some time to reach the goal
                        result = Vector2.scale(maxSpeed, result);
                    } else {
                        // goal will be reached in the next time step
                        double speed = distanceToGoal/maxSpeed;
                        result = Vector2.scale((float)speed, result);
                    }

                    simulator.setAgentPrefVelocity(i, result);
                } else {
                    Vector2 toGoalVelocityVector = new Vector2(
                            goals[i].x - currentPosition.x_, goals[i].y - currentPosition.y_);
                    float maxSpeed = simulator.getAgentMaxSpeed();
                       result = Vector2.scale(maxSpeed, toGoalVelocityVector);
                    simulator.setAgentPrefVelocity(i, result);
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

    private ArrayList<Vector2> regionToVectorList(Rectangle rect) {
        ArrayList<Vector2> obstacle = new ArrayList<Vector2>();

        Vector2 p1 = new Vector2(rect.getCorner1());
        Vector2 p3 = new Vector2(rect.getCorner2());
        Vector2 p2 = new Vector2(p1.x(), p3.y());
        Vector2 p4 = new Vector2(p3.x(), p1.y());

        obstacle.add(p1);
        obstacle.add(p4);
        obstacle.add(p3);
        obstacle.add(p2);

        return obstacle;
    }

}
