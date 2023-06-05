import java.util.*;

public class Graph<T>
{
    private final HashMap<T, List<Edge<T>>> adjacencyList;

    public final int NOT_VISITED = Integer.MAX_VALUE;

    public Graph(List<Edge<T>> edges)
    {
        adjacencyList = new HashMap<>();

        for (Edge<T> edge : edges)
        {
            adjacencyList.putIfAbsent(edge.getNode1(), new ArrayList<>());
            adjacencyList.putIfAbsent(edge.getNode2(), new ArrayList<>());

            adjacencyList.get(edge.getNode1()).add(edge);
            adjacencyList.get(edge.getNode2()).add(edge);
        }
    }

    public Map<T, Integer> calculateShortestPaths(T startNode) throws NoSuchElementException
    {
        if (!adjacencyList.containsKey(startNode))
        {
            throw new NoSuchElementException();
        }

        HashMap<T, Integer> distances = new HashMap<>();
        PriorityQueue<NodeDistancePair<T>> priorityQueue = new PriorityQueue<>();

        for (T node : adjacencyList.keySet())
        {
            distances.put(node, NOT_VISITED);
        }

        distances.put(startNode, 0);
        priorityQueue.add(new NodeDistancePair<>(startNode, 0));

        while (!priorityQueue.isEmpty())
        {
            NodeDistancePair<T> currentPair = priorityQueue.poll();
            T currentNode = currentPair.getNode();
            int currentDistance = currentPair.getDistance();

            if (currentDistance > distances.get(currentNode))
            {
                continue;
            }

            for (Edge<T> edge : adjacencyList.get(currentNode))
            {
                T neighbor = getNeighbor(currentNode, edge);

                int newDistance = currentDistance + edge.getDistance();

                if (newDistance < distances.get(neighbor))
                {
                    distances.put(neighbor, newDistance);
                    priorityQueue.add(new NodeDistancePair<>(neighbor, newDistance));
                }
            }
        }

        distances.remove(startNode);

        return distances;
    }

    public Integer calculateShortestPath(T startNode, T endNode) throws NoSuchElementException
    {
        if (!adjacencyList.containsKey(endNode))
        {
            throw new NoSuchElementException();
        }

        Map<T, Integer> distances = calculateShortestPaths(startNode);
        return distances.get(endNode);
    }

    private T getNeighbor(T node, Edge<T> edge)
    {
        return node.equals(edge.getNode1()) ? edge.getNode2() : edge.getNode1();
    }

    private static class NodeDistancePair<T> implements Comparable<NodeDistancePair<T>>
    {
        private final T node;
        private final int distance;

        public NodeDistancePair(T node, int distance)
        {
            this.node = node;
            this.distance = distance;
        }

        public T getNode()
        {
            return node;
        }

        public int getDistance()
        {
            return distance;
        }

        @Override
        public int compareTo(NodeDistancePair<T> other)
        {
            return Integer.compare(distance, other.distance);
        }
    }
}