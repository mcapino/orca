package rvolib;

import org.jgrapht.alg.DijkstraShortestPaths;
import org.jgrapht.alg.VisibilityGraphPlanner;
import org.jgrapht.util.Goal;
import rrt.JointWaypointState;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.discretization.LazyGrid;
import tt.euclid2i.region.Rectangle;
import tt.euclid2i.util.Util;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;
import tt.jointtraj.solver.SearchResult;

import java.util.*;
import java.util.Map.Entry;

public class RVOSolver {

	/** Somewhere near goal the agent must adjust the preferred velocity to avoid overshooting **/
    private float adjustPrefferedVolcityNearGoalEpsilon = 1;
    /**	The agent is considered at goal if the distance between its current position and the goal is within this tolerance **/
    private float goalReachedToleranceEpsilon; // 0.1f;

    private ArrayList<Vector2> goals;
    private float graphRadius;

    private outerLoopControl preferredVelocityControlerMethod = outerLoopControl.VISIBILITY_GRAPH;
    private VisibilityGraphPlanner visibilityGraphPlanner;
    private Collection<Region> obstacles;
    private Collection<Region> inflatedObstacles;
    private int iteration;
    //private EarliestArrivalProblem problem;
    private Collection<Region> lessInflatedObstacles;
    private float jointCost = 0;

    public RVOSolver(ArrayList<Vector2> goals,
                     Collection<Region> obstacles, Collection<Region> inflatedObstacles,
                     EarliestArrivalProblem problem) {
        this.goals = goals;
        if (goals != null) {
            for (int i = 0; i < Simulator.getInstance().getNumAgents(); i++) {
                Simulator.getInstance().getAgent(i).goal_ = goals.get(i)
                        .convertToPoint2i();
            }
        }
        this.graphRadius = 1000;// 2 * conflictRadius + 50;
        // VisManager.registerLayer(SimulationControlLayer.create(this));
        this.obstacles = obstacles;
        this.inflatedObstacles = inflatedObstacles;
        this.lessInflatedObstacles = Util.inflateRegions(problem.getObstacles(), problem.getBodyRadius(0) - 1);

        this.goalReachedToleranceEpsilon = (float) (Math.ceil((double) problem.getBodyRadius(0) / 5));
        this.adjustPrefferedVolcityNearGoalEpsilon = (float) (Math.ceil((double) problem.getBodyRadius(0) / 5));
//		System.out.println(goal_epsilon1 + "," + goal_epsilon2);
    }

    public SearchResult solve(int iterationLimit, long interruptAtNs, double maxJointCost) {

        tt.euclid2i.EvaluatedTrajectory[] trajs = new tt.euclid2i.EvaluatedTrajectory[Simulator
                .getInstance().getNumAgents()];

        for (Vector2 goal : goals) {
            Point p = goal.convertToPoint2i();
            for (Region r : obstacles) {
                if (r.isInside(p)) {
                    // System.out.println("RVO FAILURE");
                    return new SearchResult(trajs, false);
                }
            }
        }

        switch (preferredVelocityControlerMethod) {
            case LAZY_GRID:
                createGridGraph();
                break;
            case VISIBILITY_GRAPH:
                createVisibilityGraph();
                break;
        }

        for (int i = 0; i < Simulator.getInstance().getNumAgents(); ++i) {
            Simulator.getInstance().getAgent(i).clearTrajectory();
            Simulator.getInstance().getAgent(i).getCloseList().clear();
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
            Simulator.getInstance().doStep();

            // HARDCODED SIMULATION SPEED
            //try {
            //    Thread.sleep((long) simulationSpeed);
            //} catch (InterruptedException e) {
            //    e.printStackTrace();
            //}

            iteration++;
//			 System.out.println(iteration);

            calculateJointCost();
//			System.out.println("threshold: " + maxJointCost + " optimal solution cost: " + OptimalSolutionProvider.getInstance().getOptimalSolutionCost() + " cost: " + jointCost);

            if (System.nanoTime() > interruptAtNs) {

                return new SearchResult(trajs, false);
            }

        } while (!reachedGoal() && iteration < iterationLimit && jointCost <= maxJointCost);

        for (int i = 0; i < trajs.length; i++) {
            trajs[i] = Simulator.getInstance().getAgent(i)
                    .getEvaluatedTrajectory();
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
        for (int i = 0; i < Simulator.getInstance().getNumAgents(); ++i) {
            Point p = Simulator.getInstance()
                    .getAgentPosition(i).convertToPoint2i();
            Point g = goals.get(i).convertToPoint2i();
            if (!p.equals(g)) {
                jointCost += Simulator.getInstance().getTimeStep();
            }
        }
    }

    private void createVisibilityGraph() {


        for (int i = 0; i < Simulator.getInstance().getNumAgents(); ++i) {
            visibilityGraphPlanner = new VisibilityGraphPlanner(inflatedObstacles);
            visibilityGraphPlanner.createVisibilityGraph(goals.get(i).convertToPoint2i());
            visibilityGraphPlanner.evaluateGraph(i, 10 * graphRadius,
                    goals.get(i).convertToPoint2i());

        }
    }

    private void createGridGraph() {
        for (int i = 0; i < Simulator.getInstance().getNumAgents(); ++i) {
            LazyGrid grid = new LazyGrid(goals.get(i).convertToPoint2i(),
                    obstacles, new Rectangle(new Point(-1000, -1000),
                    new Point(1000, 1000)),
                    LazyGrid.PATTERN_4_WAY_WAIT, 1/*
                                                 * 2 *
												 * (int)Simulator.getInstance
												 * ().defaultAgent_.radius_
												 */);
            Simulator.getInstance().getAgent(i).setGraph(grid);
            System.out.println("agent" + i + " grid created, goal: "
                    + goals.get(i));
            DijkstraShortestPaths<Point, Line> dijkstra = new DijkstraShortestPaths<Point, Line>(
                    grid, goals.get(i).convertToPoint2i(), new Goal<Point>() {
                @Override
                public boolean isGoal(Point current) {
                    return false;
                }
            }, graphRadius);

            HashMap<Point, Double> lengths = dijkstra
                    .findLengths(100000000000l);
            Simulator.getInstance().getAgent(i).setEvaluatedGraph(lengths);
            System.out.println("agent" + i + " grid EVALUATED");
        }
    }

    private void setLazyGridPreferredVelocities() {
        for (int i = 0; i < Simulator.getInstance().getNumAgents(); ++i) {
            Vector2 p = Simulator.getInstance().getAgentPosition(i);

            if (p.convertToPoint2i().distance(goals.get(i).convertToPoint2i()) < adjustPrefferedVolcityNearGoalEpsilon) {
                Simulator.getInstance().setAgentPrefVelocity(i,
                        Vector2.minus(goals.get(i), p));
            } else {
                // c..center, u..up, d..down
                ArrayList<Point> neighbourPoints = new ArrayList<>();

                neighbourPoints.add(new Point((int) Math.floor(p.x()),
                        (int) Math.ceil(p.y()))); // cu
                neighbourPoints.add(new Point((int) Math.ceil(p.x()),
                        (int) Math.floor(p.y()))); // uc
                neighbourPoints.add(new Point((int) Math.ceil(p.x()),
                        (int) Math.ceil(p.y()))); // uu
                neighbourPoints.add(new Point((int) Math.floor(p.x()),
                        (int) Math.floor(p.y()))); // uu

                Point center = getBestPrefferedPoint(i, neighbourPoints);
                if (!center.equals(goals.get(i).convertToPoint2i())) {
                    neighbourPoints = new ArrayList<>();
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

                Vector2 graphPointVector = Vector2.minus(result, Simulator
                        .getInstance().getAgentPosition(i));
                // if (RVOMath.absSq(graphPointVector) > 1.0f) {
                graphPointVector = RVOMath.normalize(graphPointVector);
                // }
                Simulator.getInstance().setAgentPrefVelocity(i,
                        graphPointVector);
            }
        }
    }

    private void setVisibilityGraphPrefferedVelocities() {
        for (int i = 0; i < Simulator.getInstance().getNumAgents(); ++i) {

            Vector2 currentPosition = Simulator.getInstance().getAgentPosition(i);

            if (currentPosition.convertToPoint2i().distance(goals.get(i).convertToPoint2i()) < adjustPrefferedVolcityNearGoalEpsilon) {
            	// TODO compute velocity that gets agent directly to the goal at the next step.
            	Vector2  directionToGoal = Vector2.minus(goals.get(i), currentPosition);
            	// TODO continuo here
            	Simulator.getInstance().setAgentPrefVelocity(i, new Vector2(0, 0));
            } else {
                HashMap<Point, Double> evaluatedGraph = Simulator.getInstance()
                        .getAgent(i).getEvaluatedGraph();

                Iterator<Entry<Point, Double>> it = evaluatedGraph.entrySet()
                        .iterator();
                double minDistance = Double.MAX_VALUE;
                Vector2 result = null;
                Point resPoint = null;
                while (it.hasNext()) {
                    Map.Entry pairs = (Map.Entry) it.next();
                    double value = (double) pairs.getValue();
                    Point point = (Point) pairs.getKey();

                    Vector2 vector = new Vector2(point.x - currentPosition.x(), point.y
                            - currentPosition.y());
                    double dist = vector.getLength();
                    double sumDistance = dist + value;
                    double navigationEps = 0.1;

                    if (dist < navigationEps
                            && !Simulator.getInstance().getAgent(i)
                            .getCloseList().contains(point)) {
                        Simulator.getInstance().getAgent(i).getCloseList()
                                .add(point);

                    }

                    if (sumDistance < minDistance
                            && !Simulator.getInstance().getAgent(i)
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
                            resPoint = point;

                        }
                    }
                }

                if (result != null) {
                    result = RVOMath.normalize(result);
                    float maxSpeed = Simulator.getInstance().getAgentMaxSpeed();
                    result = Vector2.scale(maxSpeed, result);
                    Simulator.getInstance().setAgentPrefVelocity(i, result);
                } else {
                    Vector2 toGoalVelocity = new Vector2(
                            goals.get(i).x_ - currentPosition.x_, goals.get(i).y_ - currentPosition.y_);
                    float maxSpeed = Simulator.getInstance().getAgentMaxSpeed();
                    result = Vector2.scale(maxSpeed, toGoalVelocity);
                    Simulator.getInstance().setAgentPrefVelocity(i, result);

                }
            }
        }
    }

    private Point getBestPrefferedPoint(int i, ArrayList<Point> neighbourPoints) {
        HashMap<Point, Double> evaluatedGraph = Simulator.getInstance()
                .getAgent(i).getEvaluatedGraph();
        Double[] values = new Double[neighbourPoints.size()];
        Point p = Simulator.getInstance().getAgentPosition(i)
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
            return goals.get(i).convertToPoint2i();
        }
        return (neighbourPoints.get(index));
    }

    private boolean reachedGoal() {
        /* Check if all agents have reached their goals. */
        float eps = 1f;
        for (int i = 0; i < Simulator.getInstance().getNumAgents(); ++i) {
            Point p = Simulator.getInstance()
                    .getAgentPosition(i).convertToPoint2i();
            Point g = goals.get(i).convertToPoint2i();

            if (p.distance(g) > goalReachedToleranceEpsilon) {
                return false;
            }
        }

        return true;
    }

    public static enum outerLoopControl {
        LAZY_GRID, VISIBILITY_GRAPH;
    }

    /**
     * called from RRT algorithm
     *
     * @param from
     * @param to
     */
    public void setupProblem(JointWaypointState from, JointWaypointState to) {
        goals = new ArrayList<Vector2>();
        for (int i = 0; i < Simulator.getInstance().getNumAgents(); i++) {
            Simulator.getInstance().getAgent(i).position_ = new Vector2(
                    from.getPosition(i));
            Simulator.getInstance().getAgent(i).goal_ = to.getPosition(i);
            goals.add(new Vector2(to.getPosition(i)));
        }
    }

    public int getIterations() {
        return iteration;
    }

}
