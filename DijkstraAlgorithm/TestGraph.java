import model.Edge;
import model.Graph;
import model.Vertex;
import org.junit.Test;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;


public class TestGraph {

    private List<Vertex> nodes;
    private List<Edge> edges;
    Graph graph = initGraph();
    DijkstraAlgorithm dijkstra = new DijkstraAlgorithm(graph);
    int visited[] = new int[nodes.size()];
    @Test
    public void testGraph() {

//       The distance of the route A-B-C.
       char[] ABC = {'A','B','C'};
        printDirectDistance(ABC);

//       The distance of the route A-D.
        char[] AD = {'A','D'};
        printDirectDistance(AD);

//       The distance of the route A-D-C.
        char[] ADC = {'A','D','C'};
        printDirectDistance(ADC);

//        The distance of the route A-E-B-C-D.
        char[] AEBCD = {'A','E','B','C','D'};
        printDirectDistance(AEBCD);

//        The distance of the route A-E-D.
        char[] AED = {'A','E','D'};
        printDirectDistance(AED);

//        The number of trips starting at C and ending at C with a maximum of 3 stops.  In the sample data below, there are two such trips: C-D-C (2 stops). and C-E-B-C (3 stops).
        getCircleTripsNumberByStop('C',3);
//        The number of trips starting at A and ending at C with exactly 4 stops.  In the sample data below, there are three such trips: A to C (via B,C,D); A to C (via D,C,D); and A to C (via D,E,B).
        getPathOfLength4('A','C');
//        The length of the shortest route (in terms of distance to travel) from A to C.
        getShortestRoute('A','C');

//        The length of the shortest route (in terms of distance to travel) from B to B.
        getShortestCircle('B');
//        The number of different routes from C to C with a distance of less than 30.  In the sample data, the trips are: CDC, CEBC, CEBCDC, CDCEBC, CDEBC, CEBCEBC, CEBCEBCEBC.
        getCircuitsLessThan30('C');
    }

    /**
     * add nodes to edge
     * @param laneId
     * @param sourceLocNo
     * @param destLocNo
     * @param duration
     */
    private void addLane(String laneId, int sourceLocNo, int destLocNo,
                         int duration) {
        Edge lane = new Edge(laneId,nodes.get(sourceLocNo), nodes.get(destLocNo), duration );
        edges.add(lane);
    }

    /**
     * get edge weight
     * @param start
     * @param end
     * @return
     */
    private int getWeight(Vertex start, Vertex end) {
        if (start.equals(end)){
            return 0;
        }
        for (Edge edge : edges) {
            if (edge.getSource().equals(start) && edge.getDestination().equals(end)){
                return  edge.getWeight();
            }
        }
       return  -1;
    }

    /**
     * initialize graph
     * @return
     */
    private Graph initGraph(){
        nodes = new ArrayList<Vertex>();
        edges = new ArrayList<Edge>();
        //initialize nodes
        for (char alphabet = 'A'; alphabet <= 'E'; alphabet++) {
            Vertex location = new Vertex("Node_" + alphabet, "Node_" + alphabet);
            nodes.add(location);
        }
        //initialize edges
        addLane("Edge_AB", 'A'-65 ,'B'-65, 5);
        addLane("Edge_BC", 'B'-65, 'C'-65, 4);
        addLane("Edge_CD", 'C'-65, 'D'-65, 8);
        addLane("Edge_DC", 'D'-65, 'C'-65, 8);
        addLane("Edge_DE", 'D'-65, 'E'-65, 6);
        addLane("Edge_AD", 'A'-65, 'D'-65,5);
        addLane("Edge_CE", 'C'-65, 'E'-65,2);
        addLane("Edge_EB", 'E'-65, 'B'-65, 3);
        addLane("Edge_AE", 'A'-65, 'E'-65, 7);
        //instantiation graph
        Graph graph = new Graph(nodes, edges);
        return graph;

    }

    /**
     * get path from start node to end node
     * @param graph
     * @param start
     * @param end
     * @return
     */
    private LinkedList<Vertex> getPath(Graph graph,Vertex start, Vertex end){

        dijkstra.execute(start);
        LinkedList<Vertex> path = dijkstra.getPath(end);
        return  path;
    }

    /**
     * get distance of the path
     * @param path
     * @return
     */
    private int getDistance(LinkedList<Vertex>  path){
        int distance = 0;
        Vertex prev = path.get(0);
        for (Vertex vertex : path) {
            distance += getWeight(prev,vertex);
            prev = vertex;
        }
        return  distance;
    }

    /**
     *get distance for a path with any bridge node
     * @param path
     */
    private void printFullDistance(char[] path){
        int distance = 0;
        LinkedList<Vertex> totalPath = new LinkedList<>();
        for (int i = 0; i <= path.length-2; i++) {
            LinkedList<Vertex> tempPath = getPath(graph,nodes.get(path[i]-65),nodes.get(path[i+1]-65));
            if (tempPath.size()<1) {
                System.out.println("NO SUCH ROUTE: "+tempPath);
                return;
            }
            totalPath.addAll(tempPath);
            if (i != path.length - 2){
                totalPath.removeLast();
            }
            int tempDistance = getDistance(tempPath);
            distance += tempDistance;
        }
        System.out.println("path: "+path.toString()+" shortest route:"+totalPath+" distance: "+distance);
    }


    private void getShortestRoute(char start, char end){

        LinkedList<Vertex> shortestRoute = getPath(graph,nodes.get(start-65),nodes.get(end-65));
        if (shortestRoute.size()<1) {
            System.out.println("NO  ROUTE: FROM "+start+" TO"+ end);
            return;
        }
        int tempDistance = getDistance(shortestRoute);
        System.out.println("from "+start+" to "+end+" shortest route:"+shortestRoute+" distance: "+tempDistance);
    }

    /**
     * get shortest circle for a node
     * @param origin
     */
    private void getShortestCircle(char origin){
        int shorestDistance = Integer.MAX_VALUE;
        Vertex originNode =  new Vertex("Node_" + origin, "Node_" + origin);
        LinkedList<Vertex> shortestRoute = new LinkedList<>();
        for (Vertex node : nodes) {
            if (!node.equals(originNode)){
                LinkedList<Vertex> positiveRoute = getPath(graph,originNode,node);
                if (positiveRoute == null) {
                    continue;
                }
                LinkedList<Vertex> negativeRoute = getPath(graph,node,originNode);
                if (negativeRoute == null) {
                    continue;
                }
                if (shorestDistance > getDistance(positiveRoute)+getDistance(negativeRoute)){
                    shortestRoute.clear();
                    shorestDistance = getDistance(positiveRoute)+getDistance(negativeRoute);
                    shortestRoute.addAll(positiveRoute);
                    shortestRoute.removeLast();
                    shortestRoute.addAll(negativeRoute);
                }
            }
        }

        System.out.println(" shortest circle :"+shortestRoute+" distance: "+shorestDistance);
    }

    private  void getCircleTripsNumberByStop(char origin ,int stop){
        int number = 0;
        int shorestDistance;
        Vertex originNode =  new Vertex("Node_" + origin, "Node_" + origin);
        LinkedList<Vertex> shortestRoute = new LinkedList<>();
        for (Vertex node : nodes) {
            if (!node.equals(originNode)){
                int weight = getWeight(originNode,node);
                if (weight == -1) {
                    continue;
                }
                LinkedList<Vertex> negativeRoute = getPath(graph,node,originNode);
                if (negativeRoute == null) {
                    continue;
                }
                    shortestRoute.clear();
                    shorestDistance = weight+getDistance(negativeRoute);
                    shortestRoute.add(originNode);
                    shortestRoute.addAll(negativeRoute);

                if (shortestRoute.size() <= stop + 1){
                    number += 1;
                    //System.out.println(" shortest circle :"+shortestRoute+" distance: "+shorestDistance);
                }
            }
        }
        System.out.println(" number of circle trips for node "+originNode+"with a maximum of "+stop+" stops :"+number);
    }

    /**
     * get direct distance for a path without any bridge node
     * @param path
     */
    private void printDirectDistance(char[] path){
        LinkedList<Vertex> vertexPath = new LinkedList<>();
        for (int i = 0; i < path.length; i++) {
            vertexPath.add(nodes.get(path[i]-65));
        }
        int distance = 0;
        Vertex prev = vertexPath.get(0);
        for (Vertex vertex : vertexPath) {
            int tempWeight = getWeight(prev,vertex);
            if (tempWeight == -1){
                System.out.println("NO SUCH ROUTE: "+vertexPath);
                return;
            }else {
                distance += tempWeight;
            }
            prev = vertex;
        }
        System.out.println("path:"+vertexPath+" distance: "+distance);
    }

    /**
     * get path whose stops is exactly 4
     * @param start
     * @param end
     */
    private void getPathOfLength4(char start ,char end){
        int number = 0;
        Vertex startNode =  new Vertex("Node_" + start, "Node_" + start);
        Vertex endNode =  new Vertex("Node_" + end, "Node_" + end);

        List<Vertex> startSubs  = getSubsequents(startNode);
        List<Vertex> endPres  = getPrecursors(endNode);
        for (Vertex starNeibor : startSubs) {
            for (Vertex endNeibor : endPres) {
                LinkedList<Vertex> middleRoute;
                if (starNeibor.equals(endNeibor)){
                    if (ifCircleWithOneNode(starNeibor)){
                        //System.out.println("circle node ----------"+starNeibor);
                        number += 1;
                    }
                }else{
                    middleRoute = getPath(graph,starNeibor,endNeibor);
                    if (middleRoute != null && middleRoute.size() == 3){
                        //System.out.println("middleRoute ----------"+middleRoute);
                        number += 1;
                    }
                }


            }
        }

        System.out.println(" number of path whose length is 4  from  "+start+" to "+end+ ": "+number);
    }

    /**
     * get subsequent neighbors
     * @param node
     * @return
     */
    private List<Vertex> getSubsequents(Vertex node) {
        List<Vertex> neighbors = new ArrayList<>();
        for (Edge edge : edges) {
            if (edge.getSource().equals(node)) {
                neighbors.add(edge.getDestination());
            }
        }
        return neighbors;
    }

    /**
     * get precursor neighbors
     * @param node
     * @return
     */
    private List<Vertex> getPrecursors(Vertex node) {
        List<Vertex> precursors = new ArrayList<>();
        for (Edge edge : edges) {
            if (edge.getDestination().equals(node)) {
                precursors.add(edge.getSource());
            }
        }
        return precursors;
    }

    /**
     * if the circuit length equals 3
     * @param origin
     * @return
     */
    private boolean ifCircleWithOneNode(Vertex origin){
        for (Vertex node : nodes) {
            if (!node.equals(origin)){
                if (getSubsequents(node).contains(origin) && getPrecursors(node).contains(origin)){
                    return true;
                }
            }
        }
        return  false;
    }


    /**
     * get circuit whose distance is less than 30
     * @param origin
     */
    private void getCircuitsLessThan30(char origin){
        int circuitsLessThan30 = 0;
        //store simple circuit distance
        List<Integer> simpleDistance = new ArrayList<>();
        // store multiple circuit distance
        List<Integer> multipleDistance ;
        List<Integer> backupDistance = new ArrayList<>();
        List<LinkedList<Vertex>> simpleCircuits = getSimpleCircuits(origin);
        for (LinkedList<Vertex> simpleCircuit : simpleCircuits) {
            if (getDistance(simpleCircuit) < 30){
                simpleDistance.add(getDistance(simpleCircuit));
                circuitsLessThan30 += 1;
            }
        }
        multipleDistance = simpleDistance;
        boolean change ;
        for (int i = 0; i < Integer.MAX_VALUE; i++) {
            change = false;
            for (int j = 0; j < multipleDistance.size(); j++) {
                for (int k = 0; k < simpleDistance.size(); k++) {
                    int dis = multipleDistance.get(j) + simpleDistance.get(k);
                    if (dis < 30){
                        backupDistance.add(multipleDistance.get(j) + simpleDistance.get(k));
                        circuitsLessThan30 += 1;
                        change = true;
                    }
                }
            }
            if (!change){
                //no distance less than 30 in this round
                break;
            }
            multipleDistance = backupDistance;
            backupDistance = new ArrayList<>();
        }

        System.out.println(" number of circles for "+origin+" with a distance of less than 30 : "+circuitsLessThan30);
    }


    /**
     * get all simple circuit
     * @param origin
     */
    private List<LinkedList<Vertex>>  getSimpleCircuits(char origin){
        char path[] = new char[nodes.size()];
        for (int i = 0; i < nodes.size(); i++) {
            visited[i] = 0;
        }
        List<LinkedList<Vertex>> simpleCircuits = new ArrayList<>();
        dfs(origin,origin,path,-1,simpleCircuits);

        return simpleCircuits;
    }



    /**
     * depth first search
     * @param start
     * @param end
     * @param path
     * @param d
     * @param simpleCircuits
     */
    private  void dfs (char  start,char end,char path[],int d, List<LinkedList<Vertex>> simpleCircuits){
        Vertex startNode =  new Vertex("Node_" + start, "Node_" + start);
        Vertex endNode =  new Vertex("Node_" + end, "Node_" + end);
        List<Vertex> subs = getSubsequents(startNode);
        visited[start-65]=1;
        d++;
        path[d]=start;
        for (int i = 0; i < subs.size(); i++) {
            if (subs.get(i).equals(endNode) && d > 0){
                //find a path
                LinkedList<Vertex> nodelist = new LinkedList<>();
                for (char c : path) {
                    if ( c =='\0'){
                        continue;
                    }
                    nodelist.add(new Vertex("Node_" + c, "Node_" + c));
                }
                nodelist.add(new Vertex("Node_" + end, "Node_" + end));
                simpleCircuits.add(nodelist);
            }

            if (visited[subs.get(i).getChar()-65] != 1){
                dfs(subs.get(i).getChar(),end,path,d,simpleCircuits);
            }
        }

        //reset value after out of the loop
        visited[start-65]=0;
        path[d]= '\0';
    }
}