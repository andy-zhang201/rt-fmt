//#define _FIX_SEED_
#define _DEBUG_0_ // Draw tree
#define _DEBUG_2_ // Draw spheres

#define _DRAW_PATH
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEditor;
using Utils;
using UnityEngine.Assertions;
using System.IO;
using System;
using CsvHelper;
using System.Globalization;
using UnityEngine.SceneManagement;

using RTFMT_MultiAgent;

public class RTFMT_example_multi : MonoBehaviour
{
    // Planner parameters
    public int numberOfSamples = 500;
    public bool drawTree = false;
    public bool drawNodes = false;
    public bool fixSeed = true;

    //debug variables
    float debug_radius = 0.3f;
    Vector3 startPosition;
    //Game variables
    public Vector2 xBound = new Vector2(-300, 300);
    public Vector2 yBound = new Vector2(-300, 300);
    // public Vector2 xBound = new Vector2(-15, 15);
    // public Vector2 yBound = new Vector2(-15, 15);
    public static float ballRadius;
    //public static Vector3 offset3D = new Vector3(0, ballRadius, 0);

    public LayerMask fixedObstaclesLayer;
    public LayerMask dynamicObstaclayer;

    public LayerMask agentLayer;

    RTFMTPlannerMultiAgent planner;
    MotionController motionController;
    List<GameObject> dynamicObstaclesObjs;
    List<Transform> dynamicObstacles;
    List<Transform> agents;

    //Setpoint variables
    List<Node> path;
    Vector3 sp;

    public Vector3 goalPosition = new Vector3(0, ballRadius, 18);
    public float safeRadiusDObstacle = 2f;
    public float checkDObstacleDistance = 10f;

    public float gain = 10.0f;

    //// Priority System ////
    public float priorityDistance;
    public Node next_node;
    public Dictionary<GameObject, Pair> nextNodeDictionary;

    //// Experiments
    
    List<ResultsManagerShort> resultsListShort;
    public bool arrivedFinish = false;
    public int runCounter = 1;
    public int maxRuns = 25;

    // Exp Dta:

    long arrivalTime;
    float plannedCost;
    bool success;
    float executedCost;
    int nodeCount;
    int attempts;
    bool finishedExperiment;
    bool planTimeObtained = false;
    bool collided = false;

    // Use this for initialization
    void Start()
    {
        if(fixSeed)
		    UnityEngine.Random.InitState(42); //44

        ballRadius = StateManager.instance.ballRadius;

        fixedObstaclesLayer = StateManager.instance.fixedObstaclesLayer;
        dynamicObstaclayer = StateManager.instance.dynamicObstaclesLayer;
        dynamicObstaclesObjs = StateManager.instance.dynamicObstaclesObjects;
        dynamicObstacles = StateManager.instance.GetDynamicObstacles();
        dynamicObstacles.Remove(this.transform); // Remove itself in case the robot is also a dynamic obstacle

        Debug.Log("GameObject: " + gameObject);

        Debug.Log("Object ID: " + this.GetInstanceID());

        agents = StateManager.instance.GetAgents();
        Debug.Log("Agents Size: " + agents.Count);
        agents.Remove(this.transform);
        Debug.Log("Agents Size AFTER REMOVE: " + agents.Count);
        agentLayer = StateManager.instance.agentsLayer;

        dynamicObstacles = dynamicObstacles.Concat(agents).ToList();
        Debug.Log("Dynamic Obs Size: " + dynamicObstacles.Count);

        // planner = new RTFMTPlanner(xBound, yBound, fixedObstaclesLayer, 2, this, dynamicObstacles, ballRadius, safeRadiusDObstacle, checkDObstacleDistance); //20
        planner = new RTFMTPlannerMultiAgent(
            this,
            ballRadius,
            safeRadiusDObstacle,
            checkDObstacleDistance,
            fixedObstaclesLayer,
            dynamicObstacles);

        var rn = StateManager.instance.rn;
        var samplesList = StateManager.instance.GetSamples();
        planner.init(this.transform.position, rn, samplesList);
        goalPosition.y = ballRadius;
        planner.setGoal(goalPosition);
        //planner.setGoal(new Vector3(0, ballRadius, 13));
        //planner.setGoal(new Vector3(0, ballRadius, 10));
        //planner.setGoal(new Vector3(2, ballRadius, 1.5f));
        //planner.setGoal(planner.samplePoint());

        startPosition = this.transform.position;

        motionController = new MotionController(this, this.gain);

        Debug.Log("ID:" + this.GetInstanceID());
        resultsListShort = new List<ResultsManagerShort>();

        /////Priority System/////
        next_node = new Node(startPosition, 0, NodeState.Undefined);

    }

    // Update is called once per frame
    void Update()
    {
        Vector3 pointLeft;
        bool mouseClickedLeft = getMouseClick(out pointLeft, 0);
        Vector3 pointRight;
        bool mouseClickedRight = getMouseClick(out pointRight, 1);
        if (mouseClickedLeft)
        {
            Node selectedNode = planner.nearestNode(pointLeft);
            planner.setGoal(selectedNode);
            //startPosition = this.transform.position;
            //this.transform.position = startPosition;
        }
        if (mouseClickedRight)
        {
            Node selectedNode = planner.nearestNode(pointRight);
            planner.setRoot(selectedNode);
        }

        /////Priority System/////
        nextNodeDictionary = StateManager.instance.GetNextNodeDictionary();
        
        planner.update(gameObject, nextNodeDictionary);

        /*
		 * if (planner.isPathFound())
		{
			path = planner.getPath();
			Debug.Log("Path updated!");
		}
		*/

        path = planner.getPath();
        Node spNode = path[0];

        /////Priority System/////
        // Update Next Node
        next_node = path[0];
        priorityDistance = Vector3.Distance(this.transform.position, next_node.q);

        bool blockedNode = planner.blockedNodes.Contains(spNode);
        bool obstructedNode = planner.obstructedNodes1d.Contains(spNode);
        //if (blockedNode)
        //	this.GetComponent<Rigidbody>().velocity = Vector3.zero;
        //else if (spNode != null)
        if (spNode != null)
        {
            sp = spNode.q;
            motionController.control(sp, ballRadius / 2);
            planner.updateRoot();
        }

        if (((this.transform.position - planner.goalNode.q).magnitude <= ballRadius*2.5) && !arrivedFinish)
        {
            
            StateManager.instance.agentsDone++;
            arrivedFinish = true;
            executedCost = motionController.getExecutedCost();
            arrivalTime = planner.getPlanTime();

            Debug.Log("Agent name: "+ this.gameObject.name + " Path Cost " + executedCost + " Arrival Time: " + arrivalTime);

            StateManager.instance.totalExecutedCost += executedCost;
            StateManager.instance.totalArrivalTime += arrivalTime;
            
            //arrivalTime = planner.getPlanTime();
            //executedCost = motionController.getExecutedCost();
            //Debug.Log("planTime: " + planTime + ", arrivalTime: " + arrivalTime + ", plannedCost: " + plannedCost + ", executedCost: " + executedCost + ", success: " + success + ", collided: " + collided + ", attempts: " + attempts + ", nodes: " + nodeCount); //+ ", attempts: " + attempts + ", nodes: " + nodeCount);
            //Debug.Log("Run: #" + experimentCounter + ", Number of nodes: " + numberOfSamples);

            //this.GetComponent<Rigidbody>().velocity = Vector3.zero;
            //Vector3 newGoal = planner.sampleValidPoint();
            //startPosition = this.transform.position;
            //planner.init(this.transform.position, numberOfSamples);
            //planner.setGoal(goalPosition);

            
            //this.GetComponent<Rigidbody>().velocity = Vector3.zero;
            //this.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
            //resetDObstacles();
        }
        if (StateManager.instance.agentsDone == 3)
        {
            
            Debug.Log("Num: " +StateManager.instance.run_counter.ToString() +"Total Arrival Time: " + StateManager.instance.totalArrivalTime + " Total Executed Cost: " + StateManager.instance.totalExecutedCost);
            resultsListShort.Add(new ResultsManagerShort
            {
                experimentNum = StateManager.instance.run_counter,
                arrivalTime = StateManager.instance.totalArrivalTime,
                executedCost = StateManager.instance.totalExecutedCost,
                collision = StateManager.instance.collision_before_finish
            });

            StateManager.instance.reset_experiment();

            StateManager.instance.run_counter++;
            Reset();

        }

        // Experiment Finished
        if (StateManager.instance.run_counter > maxRuns)
        {
            // // Write Results
            String datestr = DateTime.Now.ToString("yyyy-MM-dd HH.mm.ss");
            String filename1 = String.Concat("./Results/", "MuliAgent_NoDyn", "_", datestr, ".csv");

            using (var writer = new StreamWriter(filename1))
            using (var csv = new CsvWriter(writer, CultureInfo.InvariantCulture))
            {
                csv.WriteRecords(resultsListShort);
            }
        
            // ExperimentManager experimentData = new ExperimentManager
            // {
            //     maxExperimentRuns = maxExperimentRuns,
            //     maxIterationExperiment = maxIterationExperiment,
            //     iterationIncrement = iterationIncrement,
            //     iterationExperiment = iterationExperiment,
            //     gain = gain,
            //     experiment = experiment,
            //     scenename = scenename
            //     //Samples  
            // };

            // // Write Parameters
            // List<ExperimentManager> experimentDataList = new List<ExperimentManager>();
            // experimentDataList.Add(experimentData);

            // String filename2 = String.Concat("./Results/", scenename, expname, experiment.ToString(), "_", datestr, "_param.csv");

            // using (var writer = new StreamWriter(filename2))
            // using (var csv = new CsvWriter(writer, CultureInfo.InvariantCulture))
            // {
            //     csv.WriteRecords(experimentDataList);
            // }

            // Stop the Simulation
            EditorApplication.ExecuteMenuItem("Edit/Play");

        }
        
    }

    Node findNextSetpoint(List<Node> path)
    {
        Vector3 robotPos = this.transform.position;
        Node nextNode = null;
        if (path != null)
        {
            int closestNodeIdx = -1;
            for (int i = 0; i < path.Count; i++)
            {
                float dist = (robotPos - path[i].q).magnitude;
                if (dist < ballRadius)
                    closestNodeIdx = i;
            }
            if (closestNodeIdx != -1)
            {
                int nextNodeIdx = Mathf.Min(closestNodeIdx + 1, path.Count - 1);
                nextNode = path[nextNodeIdx];
            }
        }
        return nextNode;
    }

    List<GameObject> getAllObjectsInLayer(LayerMask layer)
    {
        UnityEngine.Object[] tempList = Resources.FindObjectsOfTypeAll(typeof(GameObject));
        List<GameObject> realList = new List<GameObject>();
        GameObject temp;

        foreach (UnityEngine.Object obj in tempList)
        {
            if (obj is GameObject)
            {
                temp = (GameObject)obj;
                if ((1 << temp.layer) == layer)
                    realList.Add((GameObject)obj);
            }
        }
        return realList;
    }

    void OnDrawGizmos()
    {
        if (planner != null)
        {

            if(drawTree)
                planner.drawTree();

            if (drawNodes) {
                // Teal, small spheres
                //This is NOT the root node the robot attempts to locally reach one step at a time.
                Drawer.drawSpheres(planner.unvisitedNodes, new Color(1, 1, 1, debug_radius), 0.15f); 
                // Color(red,green,blue,opacity)
                
                //White
                Drawer.drawSpheres(planner.openNodes, new Color(1, 1, 1, 1), debug_radius); // White, large spheres

                //Black
                Drawer.drawSpheres(planner.closedNodes, new Color(0, 0, 0, 0.5f), debug_radius); // Black

                //Blue
                Drawer.drawSpheres(planner.obstructedNodes.SelectMany(list => list).ToList(), new Color(0, 0, 1, 1), debug_radius); // Blue

                //Yellow, small
                Drawer.drawSpheres(planner.blockedNodes, new Color(1, 1, 0, 0.5f), debug_radius / 1.5f); // Yellow, small

                //Green
                Drawer.drawSpheres(planner.rewireRootList, Color.green, debug_radius);

                //Dark Green
                Drawer.drawSpheres(planner.rewireLocalList, new Color(0, 0.5f, 0, 1), debug_radius); //Solid Dark green
                //Gizmos.color = new Color(1, 0.6f, 0, 1);
                //Gizmos.DrawSphere(sp, debug_radius + 0.1f);
                //drawCircle(this.transform.position, 1, Color.red);
                //drawCircle(this.transform.position, 2, Color.blue);
                Gizmos.color = new Color(0.7f, 1, 1);
                Gizmos.DrawSphere(planner.rootNode.q, debug_radius);
            }
#if _DRAW_PATH
            Gizmos.color = new Color(0.9f, 0.1f, 0.1f);
            Gizmos.DrawSphere(planner.goalNode.q, debug_radius);
            Gizmos.color = new Color(0.4f, 1f, 0.4f);
            Gizmos.DrawSphere(startPosition, debug_radius);

            planner.drawPath();
#endif
        }


    }

    bool getMouseClick(out Vector3 point, int button)
    {
        point = new Vector3();
        bool result = false;
        if (Input.GetMouseButtonDown(button))
        {
            result = true;
            RaycastHit hit;
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

            if (Physics.Raycast(ray, out hit))
            {
                Transform objectHit = hit.transform;
                if (objectHit.name == "Plane")
                {
                    point = hit.point;
                }
                // Do something with the object that was hit by the raycast.
            }
        }
        return result;
    }

    void OnCollisionEnter(Collision collision)
    {
        var tag = collision.gameObject.tag;
        if (((1 << collision.gameObject.layer) == agentLayer)|| ((1 << collision.gameObject.layer) == fixedObstaclesLayer))
        {
            Debug.Log("Collision");
            collided = true;
            StateManager.instance.collision_before_finish = true;
        }

    }

    public void Reset()
    {
    	SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }
    public void resetDObstacles()
    {
        for (int i = 0; i < dynamicObstaclesObjs.Count; i++)
        {
            dynamicObstaclesObjs[i].SendMessage("Reset");
        }


    }

}
