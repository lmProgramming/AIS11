public class Edge<T> {
    private T node1;
    private T node2;
    private int distance;

    public Edge(T node1, T node2, int distance) {
        this.node1 = node1;
        this.node2 = node2;
        this.distance = distance;
    }

    public T getNode1() {
        return node1;
    }

    public T getNode2() {
        return node2;
    }

    public int getDistance() {
        return distance;
    }
}
