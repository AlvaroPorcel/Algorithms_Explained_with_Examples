package segundoejercicio_;

import java.util.*;

public class FlowsInNetwork {
    static final int INF = Integer.MAX_VALUE;

    static class FordFulkerson {
        int vertices;
        int[][] capacity;
        List<Integer>[] adj;

        public FordFulkerson(int vertices) {
            this.vertices = vertices;
            capacity = new int[vertices][vertices];
            adj = new List[vertices];
            for (int i = 0; i < vertices; i++) {
                adj[i] = new ArrayList<>();
            }
        }

        void addEdge(int from, int to, int cap) {
            adj[from].add(to); //hay una arista que va desde el nodo from al nodo to
            adj[to].add(from); // Ford-Fulkerson trata las aristas como bidireccionales, incluso si el grafo original es dirigido. 
            capacity[from][to] = cap;
        }
        	//paths aumentativos
        boolean dfs(int source, int sink, int[] parent) {
            boolean[] visited = new boolean[vertices];
            Stack<Integer> stack = new Stack<>();
            stack.push(source);
            visited[source] = true;//source visitado

            while (!stack.isEmpty()) {
                int current = stack.pop();//se extrae el nodo current de la pila para ser procesado
                for (int next : adj[current]) {//se exploran todos sus nodos adyacentes (vecinos). 
                    if (!visited[next] && capacity[current][next] > 0) {
                    	//los nodos adyacentes que aún tienen capacidad disponible para enviar flujo
                        stack.push(next);//Agrega el nodo adyacente a la pila para continuar la búsqueda.
                        visited[next] = true;
                        parent[next] = current;
                        if (next == sink) {
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        public int fordFulkerson(int source, int sink) {
            int flow = 0;
            int[] parent = new int[vertices];//Arreglo para almacenar el camino encontrado por dfs.

            while (dfs(source, sink, parent)) {//Se ejecuta mientras haya un camino aumentante desde source hasta sink. 
            	//La función dfs busca ese camino y lo almacena en parent.
                int pathFlow = INF;
                for (int v = sink; v != source; v = parent[v]) {//hasta que no llegemos a source y actuali v
                    int u = parent[v];
                    pathFlow = Math.min(pathFlow, capacity[u][v]); //u nodo padre, v actual
                }													//la capacidad residual de la arista que conecta u con v
                //capacidades residuales
                for (int v = sink; v != source; v = parent[v]) {
                    int u = parent[v];
                    capacity[u][v] -= pathFlow;
                    capacity[v][u] += pathFlow;
                }

                flow += pathFlow;
            }
            return flow;
        }
    }

    static class MinCostMaxFlow {
        int vertices;
        int[][] capacity;
        int[][] cost;
        List<Integer>[] adj; //vertices adyacentes
        int[] dist; //distancias mas cortas nodo fuente a los demas
        int[] parent;
        boolean[] inQueue; // nodo esta en la cola

        public MinCostMaxFlow(int vertices) {
            this.vertices = vertices;
            capacity = new int[vertices][vertices];
            cost = new int[vertices][vertices];
            adj = new List[vertices];
            for (int i = 0; i < vertices; i++) {
                adj[i] = new ArrayList<>();
            }
            dist = new int[vertices];
            parent = new int[vertices];
            inQueue = new boolean[vertices];
        }

        void addEdge(int from, int to, int cap, int len) {
            adj[from].add(to);
            adj[to].add(from);
            capacity[from][to] = cap;
            cost[from][to] = len;
            cost[to][from] = -len;
        }

        boolean spfa(int source, int sink) {
            Arrays.fill(dist, INF);
            Arrays.fill(parent, -1);
            Arrays.fill(inQueue, false);
            Queue<Integer> queue = new LinkedList<>();//para almacenar los nodos que se van a explorar
            queue.add(source);
            dist[source] = 0; //, ya que la distancia desde el nodo fuente a sí mismo es cero
            inQueue[source] = true;

            while (!queue.isEmpty()) {
                int cur = queue.poll();
                inQueue[cur] = false;// esta siendo procesado asi que fuera de la cola

                for (int next : adj[cur]) {
                    if (capacity[cur][next] > 0 && dist[next] > dist[cur] + cost[cur][next]) {
               //Si la distancia actualmente conocida desde el nodo fuente hasta next es mayor 
               //que la distancia desde el nodo fuente hasta cur, más el costo de ir de cur a next
                        dist[next] = dist[cur] + cost[cur][next];
                        parent[next] = cur;//cur es el nodo previo en el camino más corto desde el nodo fuente hasta next.
                        if (!inQueue[next]) {
                            queue.add(next);
                            inQueue[next] = true;
                        }
                    }
                }
            }
            return parent[sink] != -1;//ha encontrado solucion
        }

        public int[] minCostMaxFlow(int source, int sink) {
            int flow = 0;
            int flowCost = 0;

            while (spfa(source, sink)) {
                int pathFlow = INF;
                for (int cur = sink; cur != source; cur = parent[cur]) {
                    int prev = parent[cur];
                    pathFlow = Math.min(pathFlow, capacity[prev][cur]);
                }
                //actualizar capacidades residuales de edges and reverse edges along the path 
                for (int cur = sink; cur != source; cur = parent[cur]) {
                    int prev = parent[cur];
                    capacity[prev][cur] -= pathFlow;
                    capacity[cur][prev] += pathFlow;
                    flowCost += pathFlow * cost[prev][cur];
                }
                flow += pathFlow;
            }
            return new int[]{flow, flowCost};
        }
    }

    public static void main(String[] args) {
        Scanner sc = new Scanner(System.in);
        int n = sc.nextInt();
        int m = sc.nextInt();
        
        FordFulkerson fordFulkerson = new FordFulkerson(n + 1);
        MinCostMaxFlow minCostMaxFlow = new MinCostMaxFlow(n + 1);

        for (int i = 0; i < m; i++) {
            int u = sc.nextInt();
            int v = sc.nextInt();
            int cap = sc.nextInt();
            int length = sc.nextInt();
            fordFulkerson.addEdge(u, v, cap);
            minCostMaxFlow.addEdge(u, v, cap, length);
        }

        System.out.println("Max Flow: " + fordFulkerson.fordFulkerson(1, n));

        int[] result = minCostMaxFlow.minCostMaxFlow(1, n);
        System.out.println("Max Flow with Min Cost: " + result[0]);
        System.out.println("Min Cost: " + result[1]);
    }
}

