package org.jgrapht.traverse;


import org.jgrapht.Graph;
import org.jgrapht.Graphs;
import org.jgrapht.util.FibonacciHeapNode;

import java.util.HashMap;

public class DijkstraIterator<V, E> extends ORCAClosestFirstIterator<V, E> {

    private final HashMap<V, Double> shortestPathLengths;

    public DijkstraIterator(Graph<V, E> g, V startVertex) {
        this(g, startVertex, Double.MAX_VALUE);
    }

    public DijkstraIterator(Graph<V, E> g, V startVertex, double radius) {
        super(g, startVertex, radius);
        this.shortestPathLengths = new HashMap<V, Double>();
    }

    @Override
    protected void encounterVertex(V vertex, E edge) {
        double shortestPathLength;
        double heuristicEstimate;

        if (edge == null) {
            shortestPathLength = 0; //vertex is a starting vertex
        } else {
            shortestPathLength = calculatePathLength(vertex, edge);
        }

        double key = shortestPathLength;

        FibonacciHeapNode<QueueEntry<V, E>> node = createSeenData(vertex, edge);
        putSeenData(vertex, node);
        savePathLength(vertex, shortestPathLength);
        heap.insert(node, key);
    }

    @Override
    protected void encounterVertexAgain(V vertex, E edge) {
        FibonacciHeapNode<QueueEntry<V, E>> node = getSeenData(vertex);

        if (node.getData().frozen) {
            return;
        }

        double candidatePathLength = calculatePathLength(vertex, edge);
        double candidateKey = candidatePathLength;

        if (candidateKey < node.getKey()) {
            node.getData().spanningTreeEdge = edge;
            node.getData().vertex = vertex;
            heap.decreaseKey(node, candidateKey);
            savePathLength(vertex, candidatePathLength);
        }
    }

    private double calculatePathLength(V vertex, E edge) {
        assertNonNegativeEdge(edge);

        V otherVertex = Graphs.getOppositeVertex(getGraph(), edge, vertex);
        return shortestPathLengths.get(otherVertex)
                + getGraph().getEdgeWeight(edge);
    }

    private void savePathLength(V vertex, double length) {
        shortestPathLengths.put(vertex, length);
    }

    public HashMap<V, Double> getShortestPathLengths() {
        return shortestPathLengths;
    }

    public double getShortestPathLength(V vertex) {
        return shortestPathLengths.get(vertex);
    }
}
