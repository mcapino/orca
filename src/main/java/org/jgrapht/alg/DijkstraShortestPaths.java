package org.jgrapht.alg;

import org.jgrapht.Graph;
import org.jgrapht.traverse.DijkstraIterator;
import org.jgrapht.util.Goal;

import java.util.HashMap;


public class DijkstraShortestPaths<V, E> {


    private Graph<V, E> graph;
    private V startVertex;
    private Goal<V> goal;
    private double radius;
    public long expandedStates = 0;
    public long searchRuntime = 0;


    private static final long INF = Long.MAX_VALUE;

    public static <V, E> HashMap<V, Double> findShortestPaths(Graph<V, E> graph,
                                                              V startVertex, Goal<V> goal, double radius, long timeoutNs) {

        DijkstraShortestPaths<V, E> alg = new DijkstraShortestPaths<V, E>(graph,
                startVertex, goal, radius);
        return alg.findLengths(timeoutNs);
    }

    public DijkstraShortestPaths(Graph<V, E> graph, V startVertex,
                                 Goal<V> goalChecker, double radius) {
        this.graph = graph;
        this.startVertex = startVertex;
        this.goal = goalChecker;
        this.radius = radius;
    }

    public HashMap<V, Double> findLengths(long timeoutNs) {
        DijkstraIterator<V, E> iter =
                new DijkstraIterator<V, E>(graph, startVertex, radius);

        long startTimeNs = System.nanoTime();

        while (iter.hasNext() && (System.nanoTime() - startTimeNs) < timeoutNs) {
            V vertex = iter.next();
            expandedStates++;

            if (goal.isGoal(vertex)) {
                searchRuntime += System.nanoTime() - startTimeNs;
                return iter.getShortestPathLengths();
            }
        }

        searchRuntime += System.nanoTime() - startTimeNs;
        return iter.getShortestPathLengths();
    }
}
