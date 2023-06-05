import org.junit.jupiter.api.*;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

public class GraphTests {
    private Graph<String> graph;

    @Test
    public void calculateShortestPaths_a() {
        var expected = Map.of(
                "b", 2,
                "c", 5,
                "d", 5,
                "e", 6,
                "f", 6,
                "g", 9,
                "h", 8
        );

        var actual = graph.calculateShortestPaths("a");

        assertEquals(expected, actual);
    }

    @Test
    public void calculateShortestPaths_b() {
        var expected = Map.of(
                "a", 2,
                "c", 7,
                "d", 3,
                "e", 4,
                "f", 4,
                "g", 7,
                "h", 6
        );

        var actual = graph.calculateShortestPaths("b");

        assertEquals(expected, actual);
    }

    @Test
    public void calculateShortestPaths_c() {
        var expected = Map.of(
                "a", 5,
                "b", 7,
                "d", 5,
                "e", 8,
                "f", 6,
                "g", 11,
                "h", 10
        );

        var actual = graph.calculateShortestPaths("c");

        assertEquals(expected, actual);
    }

    @Test
    public void calculateShortestPaths_d() {
        var expected = Map.of(
                "a", 5,
                "b", 3,
                "c", 5,
                "e", 3,
                "f", 1,
                "g", 6,
                "h", 5
        );

        var actual = graph.calculateShortestPaths("d");

        assertEquals(expected, actual);
    }

    @Test
    public void calculateShortestPaths_e() {
        var expected = Map.of(
                "a", 6,
                "b", 4,
                "c", 8,
                "d", 3,
                "f", 4,
                "g", 3,
                "h", 2
        );

        var actual = graph.calculateShortestPaths("e");

        assertEquals(expected, actual);
    }

    @Test
    public void calculateShortestPaths_f() {
        var expected = Map.of(
                "a", 6,
                "b", 4,
                "c", 6,
                "d", 1,
                "e", 4,
                "g", 7,
                "h", 6
        );

        var actual = graph.calculateShortestPaths("f");

        assertEquals(expected, actual);
    }

    @Test
    public void calculateShortestPaths_g() {
        var expected = Map.of(
                "a", 9,
                "b", 7,
                "c", 11,
                "d", 6,
                "e", 3,
                "f", 7,
                "h", 1
        );

        var actual = graph.calculateShortestPaths("g");

        assertEquals(expected, actual);
    }

    @Test
    public void calculateShortestPaths_h() {
        var expected = Map.of(
                "a", 8,
                "b", 6,
                "c", 10,
                "d", 5,
                "e", 2,
                "f", 6,
                "g", 1
        );

        var actual = graph.calculateShortestPaths("h");

        assertEquals(expected, actual);
    }

    @Test
    public void calculateShortestPaths_fromNotExistingNode() {
        assertThrows(NoSuchElementException.class, () -> {
            graph.calculateShortestPaths("doesn't exist");
        });
    }

    @Test
    public void calculateShortestPaths_referenceType() {
        var studentsMap = Map.of(
                "a", new Student("Alicja"),
                "b", new Student("Bernard"),
                "c", new Student("Cecylia"),
                "d", new Student("Dorota"),
                "e", new Student("Edward"),
                "f", new Student("Franciszek"),
                "g", new Student("Genowefa"),
                "h", new Student("Hanna")
        );

        var graph = createReferenceTestGraph(studentsMap);

        var expected = Map.of(
                studentsMap.get("b"), 2,
                studentsMap.get("c"), 5,
                studentsMap.get("d"), 5,
                studentsMap.get("e"), 6,
                studentsMap.get("f"), 6,
                studentsMap.get("g"), 9,
                studentsMap.get("h"), 8
        );

        var actual = graph.calculateShortestPaths(studentsMap.get("a"));

        assertEquals(expected, actual);
    }

    // nowe testy
    @Test
    public void calculateShortestPath_af() {
        var distance = graph.calculateShortestPath("a", "f");
        assertEquals(6, distance);
    }

    @Test
    public void calculateShortestPath_df() {
        var distance = graph.calculateShortestPath("d", "f");
        assertEquals(1, distance);
    }

    @Test
    public void calculateShortestPath_gc() {
        var distance = graph.calculateShortestPath("g", "c");
        assertEquals(11, distance);
    }

    @Test
    public void calculateShortestPath_startNodeDoesntExist() {
        assertThrows(NoSuchElementException.class, () -> {
            graph.calculateShortestPath("z", "c");
        });
    }

    @Test
    public void calculateShortestPath_endNodeDoesntExist() {
        assertThrows(NoSuchElementException.class, () -> {
            graph.calculateShortestPath("a", "x");
        });
    }
    // nowe testy

    @BeforeEach
    public void createTestGraph() {
        this.graph = new Graph<>(createTestEdges());
    }

    private Graph<Student> createReferenceTestGraph(Map<String, Student> studentsMap) {
        var edges = createTestEdges().stream()
                .map(oldEdge -> new Edge<Student>(
                        studentsMap.get(oldEdge.getNode1()),
                        studentsMap.get(oldEdge.getNode2()),
                        oldEdge.getDistance()))
                .collect(Collectors.toList());

        return new Graph<Student>(edges);
    }

    private List<Edge<String>> createTestEdges() {
        return Arrays.asList(
                new Edge<>("a", "b", 2),
                new Edge<>("a", "c", 5),
                new Edge<>("b", "d", 3),
                new Edge<>("b", "e", 4),
                new Edge<>("c", "d", 5),
                new Edge<>("c", "f", 6),
                new Edge<>("d", "e", 3),
                new Edge<>("d", "f", 1),
                new Edge<>("e", "f", 4),
                new Edge<>("e", "g", 8),
                new Edge<>("e", "h", 2),
                new Edge<>("f", "g", 7),
                new Edge<>("g", "h", 1)
        );
    }

    private class Student {
        String name;

        public Student(String name) {
            this.name = name;
        }
    }
}
