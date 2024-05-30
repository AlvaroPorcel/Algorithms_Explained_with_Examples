package primerejercicio_;

import java.util.*;

class Ciudad {
    int id;
    double latitude;
    double longitude;

    Ciudad(int id, double latitude, double longitude) {
        this.id = id;
        this.latitude = latitude;
        this.longitude = longitude;
    }
}

class Road {
    int v1;
    int v2;
    int distance;

    Road(int v1, int v2, int distance) {
        this.v1 = v1;
        this.v2 = v2;
        this.distance = distance;
    }
}

public class Graph_Algorithm {

    private static List<Ciudad> cities = new ArrayList<>();
    private static List<List<Road>> adjList = new ArrayList<>();
    private static int n, m;

    public static void main(String[] args) {
        Scanner sc = new Scanner(System.in);

        n = sc.nextInt();
        m = sc.nextInt();

        for (int i = 0; i < n; i++) {
        	double latitude = Double.parseDouble(sc.next());
        	double longitude = Double.parseDouble(sc.next());
            cities.add(new Ciudad(i + 1, latitude, longitude));
            adjList.add(new ArrayList<>());
        }

        for (int i = 0; i < m; i++) {
            int v1 = sc.nextInt();
            int v2 = sc.nextInt();
            int distance = sc.nextInt();
            adjList.get(v1 - 1).add(new Road(v1, v2, distance));
            adjList.get(v2 - 1).add(new Road(v2, v1, distance));
        }

        int start = sc.nextInt();
        int end = sc.nextInt();

        
        List<Integer> bfsPath = bfs(start, end);
        System.out.println(bfsPath.size() - 2);
        for (int city : bfsPath) {
            System.out.print(city + " ");
        }
        System.out.println();

        List<Integer> dijkstraPath = dijkstra(start, end);
        int dijkstraDistance = calculatePathDistance(dijkstraPath);
        System.out.println(dijkstraDistance);
        for (int city : dijkstraPath) {
            System.out.print(city + " ");
        }
        System.out.println();

        List<Integer> aStarPath = aStar(start, end);
        int aStarDistance = calculatePathDistance(aStarPath);
        System.out.println(aStarDistance);
        for (int city : aStarPath) {
            System.out.print(city + " ");
        }
        System.out.println();

        sc.close();
    }

    private static List<Integer> bfs(int start, int end) {
        Queue<Integer> queue = new LinkedList<>();
        boolean[] visited = new boolean[n];
        int[] prev = new int[n];//se utiliza para rastrear el predecesor de cada nodo visitado
        Arrays.fill(prev, -1);

        queue.add(start);
        visited[start - 1] = true;//visitado start

        while (!queue.isEmpty()) {
            int node = queue.poll();
            if (node == end) {//nodo final entonces se acaba
                break;
            }
            for (Road road : adjList.get(node - 1)) {//ciudad recorre
                if (!visited[road.v2 - 1]) {
                    queue.add(road.v2);
                    visited[road.v2 - 1] = true;
                    prev[road.v2 - 1] = node;//el nodo predecesor del vecino como actual nodo
                }
            }
        }

        List<Integer> path = new ArrayList<>();
        //El valor -1 indica que se ha alcanzado el inicio del camino o que no hay predecesor 
        for (int at = end; at != -1; at = prev[at - 1]) {//actualiza el predecesor del nodo actual
            path.add(at);
        }
        Collections.reverse(path);
        return path;
    }

    private static List<Integer> dijkstra(int start, int end) {
        int[] dist = new int[n];
        int[] prev = new int[n];
        Arrays.fill(dist, Integer.MAX_VALUE);
        Arrays.fill(prev, -1);//no se conoce ningun predecesor
        dist[start - 1] = 0;

        PriorityQueue<int[]> pq = new PriorityQueue<>(Comparator.comparingInt(a -> a[1]));
        //toma un array a y devuelve el valor del segundo elemento (a[1]), que es la distancia mínima conocida desde el nodo de inicio hasta el nodo representado por el array
        pq.add(new int[]{start, 0});

        while (!pq.isEmpty()) {
            int[] current = pq.poll();
            //Se extrae el identificador del nodo (node) y su distancia mínima conocida (currentDist) del array current
            int node = current[0];
            int currentDist = current[1];

            if (currentDist > dist[node - 1]) {
                continue;
            }

            for (Road road : adjList.get(node - 1)) {
                int neighbor = road.v2;
                //Representa la distancia mínima conocida desde el nodo de inicio hasta el nodo actual en el bucle actua
                //Representa la distancia entre el nodo actual y su vecino a lo largo de la arista considerada en el bucle
                int newDist = dist[node - 1] + road.distance;
                if (newDist < dist[neighbor - 1]) {
                    dist[neighbor - 1] = newDist;
                    prev[neighbor - 1] = node;//camino más corto desde el nodo de inicio hasta neighbor pasa por node.
                    pq.add(new int[]{neighbor, newDist});
                }
            }
        }

        List<Integer> path = new ArrayList<>();
        for (int at = end; at != -1; at = prev[at - 1]) {
            path.add(at);
        }
        Collections.reverse(path);
        return path;
    }

    private static List<Integer> aStar(int start, int end) {
        int[] dist = new int[n];
        int[] prev = new int[n];
        Arrays.fill(dist, Integer.MAX_VALUE);
        Arrays.fill(prev, -1);
        dist[start - 1] = 0;

        PriorityQueue<int[]> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a[1]));
        pq.add(new int[]{start, 0});

        while (!pq.isEmpty()) {
            int[] current = pq.poll();//extrae y elimina el nodo con la distancia acumulada más baja de la cola de prioridad
            int node = current[0];

            if (node == end) {
                break;
            }

            for (Road road : adjList.get(node - 1)) {
                int neighbor = road.v2;
                int newDist = dist[node - 1] + road.distance;//nodo actual y distancia carretera
                if (newDist < dist[neighbor - 1]) {
                    dist[neighbor - 1] = newDist;
                    prev[neighbor - 1] = node;
                    double heuristic = newDist + heuristic(neighbor, end);
                    pq.add(new int[]{neighbor, (int) heuristic});
                }
            }
        }

        List<Integer> path = new ArrayList<>();
        for (int at = end; at != -1; at = prev[at - 1]) {
            path.add(at);
        }
        Collections.reverse(path);
        return path;
    }

    private static double heuristic(int a, int b) {
        Ciudad cityA = cities.get(a - 1);
        Ciudad cityB = cities.get(b - 1);
        double dx = cityA.latitude - cityB.latitude;
        double dy = cityA.longitude - cityB.longitude;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private static int calculatePathDistance(List<Integer> path) {
        int distance = 0;
        for (int i = 1; i < path.size(); i++) {
            int v1 = path.get(i - 1);
            int v2 = path.get(i);
            for (Road road : adjList.get(v1 - 1)) {
                if (road.v2 == v2) { // arista road conecta el nodo v1 con el nodo v2
                    distance += road.distance;
                    break;
                }
            }
        }
        return distance;
    }
}



