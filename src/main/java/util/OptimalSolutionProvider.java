package util;

import java.util.ArrayList;
import java.util.Collection;

import javax.vecmath.Vector3d;

import org.jgrapht.GraphPath;
import org.jgrapht.alg.VisibilityGraphPlanner;

import rvolib.RVOTrajectory;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;
import tt.vis.TrajectoriesLayer;
import tt.vis.TrajectoriesLayer.TrajectoriesProvider;
import cz.agents.alite.vis.VisManager;

public class OptimalSolutionProvider {

    private static OptimalSolutionProvider instance = new OptimalSolutionProvider();
    private EarliestArrivalProblem problem;
    private double agentMaxSpeed;
    private Collection<Region> lessInflatedObstacles;
    private Collection<Region> moreInflatedObstacles;
    private int optimalSolutionCost;
    private boolean isInitialized = false;
    private ArrayList<Long> extensionRunTimes;
    private EvaluatedTrajectory[] trajectories;

    private boolean showOptimalSolution = false;


    public OptimalSolutionProvider() {
    }

    public void init(EarliestArrivalProblem problem, double agentMaxSpeed, Collection<Region> lessInflatedObstacles, Collection<Region> moreInflatedObstacles) {
        trajectories = new EvaluatedTrajectory[problem.nAgents()];
        this.problem = problem;
        this.agentMaxSpeed = agentMaxSpeed;
        this.lessInflatedObstacles = lessInflatedObstacles;
        this.moreInflatedObstacles = moreInflatedObstacles;
        this.optimalSolutionCost = calculateFeasibleOptimisticSolution();
        extensionRunTimes = new ArrayList<Long>();

        if (showOptimalSolution) {
            visualizeOptimalSolution();
        }
        isInitialized = true;
    }


    public static OptimalSolutionProvider getInstance() {
        return instance;
    }

    public int getOptimalSolutionCost() {
        if (!isInitialized) {
            throw new RuntimeException("OptimalSolutionProvider used without inititalization");
        }
        return optimalSolutionCost;
    }

    private int calculateFeasibleOptimisticSolution() {

        int sumMaxTime = 0;
        ArrayList<GraphPath<Point, Line>> shortestPaths = createVisibilityGraph(
                problem.getStarts(), problem.getTargets());
        // create trajectories
        for (int i = 0; i < problem.nAgents(); i++) {
            GraphPath<Point, Line> graphPath = shortestPaths.get(i);
            trajectories[i] = calculateTrajectory(graphPath, (int) agentMaxSpeed);
        }


        for (int i = 0; i < problem.nAgents(); i++) {
            double length = 0;
            ArrayList<Point> trajectory = new ArrayList<Point>();
            if (shortestPaths.get(i) == null) {
                return -1;
            }
            Point edgeStart = shortestPaths.get(i).getStartVertex();
            for (Line edge : shortestPaths.get(i).getEdgeList()) {
                Point pp1 = edge.getStart();
                Point pp2 = edge.getEnd();
                Point edgeEnd;
                if (edgeStart.equals(pp1)) {
                    edgeEnd = pp2;
                } else {
                    edgeEnd = pp1;
                }

                length += edgeStart.distance(edgeEnd);
                edgeStart = edgeEnd;
            }
            int time = (int) Math.round(length / agentMaxSpeed);
            sumMaxTime += time;
        }
        return sumMaxTime;
    }

    private ArrayList<GraphPath<Point, Line>> createVisibilityGraph(Point[] starts, Point[] goals) {
        ArrayList<GraphPath<Point, Line>> shortestPaths = new ArrayList<GraphPath<Point, Line>>();

        for (int i = 0; i < problem.nAgents(); i++) {
            VisibilityGraphPlanner visibilityGraphPlanner = new VisibilityGraphPlanner(starts[i], goals[i], lessInflatedObstacles, moreInflatedObstacles, false);
            shortestPaths.add(visibilityGraphPlanner.getShortestPath(starts[i], goals[i]));
        }

        return shortestPaths;
    }

    public void addExtensionRunTime(Long t) {
        extensionRunTimes.add(t);
    }

    public double getAverageExtensionRunTime() {
        double avg = 0;
        for (Long l : extensionRunTimes) {
            avg += l;
        }

        return avg / extensionRunTimes.size();
    }


    private EvaluatedTrajectory calculateTrajectory(
            GraphPath<Point, Line> shortestPath, int maxSpeed) {

        ArrayList<Point> trajectory = new ArrayList<Point>();

        Point edgeStart = shortestPath.getStartVertex();
        for (Line edge : shortestPath.getEdgeList()) {
            Point pp1 = edge.getStart();
            Point pp2 = edge.getEnd();
            Point edgeEnd;
            if (edgeStart.equals(pp1)) {
                edgeEnd = pp2;
            } else {
                edgeEnd = pp1;
            }

            Vector3d velocity = new Vector3d(edgeEnd.x - edgeStart.x, edgeEnd.y
                    - edgeStart.y, 0);
            velocity.normalize();
            velocity.scale(maxSpeed);
            int stepCounts = (int) (edgeStart.distance(edgeEnd) / maxSpeed);
            for (int t = 0; t <= stepCounts; t++) {
                Vector3d addVelocity = new Vector3d(velocity);
                addVelocity.scale(t);
                Point p = new Point(edgeStart.x
                        + (int) Math.round(addVelocity.x), edgeStart.y
                        + (int) Math.round(addVelocity.y));
                trajectory.add(p);
            }
            edgeStart = edgeEnd;
        }
        EvaluatedTrajectory traj = new RVOTrajectory(trajectory, 1, shortestPath.getEndVertex(), trajectory.size());
        return traj;
    }


    private void visualizeOptimalSolution() {
        VisManager.registerLayer(TrajectoriesLayer.create(
                new TrajectoriesProvider<tt.euclid2i.Point>() {
                    @Override
                    public tt.discrete.Trajectory<tt.euclid2i.Point>[] getTrajectories() {
                        return trajectories;
                    }
                }, new tt.euclid2i.vis.ProjectionTo2d(), 1, 10000,
                6, 's'));
    }
}
