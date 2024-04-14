using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

using UnityEngine;
using UnityEditor;
using UnityEngine.Assertions;

using Utils;

//// Priority Systen ////
public struct Pair
{
    public float priorityDistance;
    public RTFMT_MultiAgent.Node next_node;
}

public class StateManager : MonoBehaviour {
    // Global state manager instance
    public static StateManager instance { get; private set; }

    /*
     ***********************
     *   State variables
     ***********************
    */

    // Sampled nodes
    public int sampleNumber = 500;
    private List<Vector3> samplesList;

    // Agents layer
    public LayerMask agentsLayer;
    private List<GameObject> agentsObjects;
    private List<Transform> agents;
    
    // Fixed obstacles layer
    public LayerMask fixedObstaclesLayer;
    
    // Dynamic obstacle positions
    public LayerMask dynamicObstaclesLayer;
    public List<GameObject> dynamicObstaclesObjects;
    private List<Transform> dynamicObstacles;

    // Agent ball radius
    public float sphereColliderRadius = 0.5f;
    public float ballRadius;

    public Dictionary<GameObject, Pair> nextNodeDictionary;
    private static Dictionary<string, int> staticPrioDictionary;

    /*
     **************************
     *   Validator variables
     **************************
    */

    // Neighbor checking variables
    int dim = 2;
    float rnScale = 1.1f;
    public float rn;

    // Map bounds
    public Vector2 xBound = new Vector2(-50, 50);
    public Vector2 yBound = new Vector2(-50, 50);

    // Create singleton class instance (only one instance present at all times)
    private void Awake() {
        if (instance != null && instance != this) {
            Destroy(this);
        } else {
            instance = this;

            /*
             ***************************
             Setup simulation state
             ***************************
            */

            // Fixed obstacles
            fixedObstaclesLayer = LayerMask.GetMask("Obstacles");
            
            // Dynamic obstacles
            dynamicObstaclesLayer = LayerMask.GetMask("Dynamic Obstacles");
            dynamicObstaclesObjects = getAllObjectsInLayer(dynamicObstaclesLayer);
            dynamicObstacles = dynamicObstaclesObjects.Select(x => x.transform).ToList();

            // Agents
            agentsLayer = LayerMask.GetMask("Agents");
            agentsObjects = getAllObjectsInLayer(agentsLayer);
            agents = agentsObjects.Select(x => x.transform).ToList();

            for (int i = 0; i < agents.Count; i++)
            {
                Debug.Log("AgentID: " + agentsObjects[i].GetInstanceID());
            }
            
            // Setup ball radius (fixed for all agents)
            ballRadius = sphereColliderRadius * Mathf.Max(transform.lossyScale.x, transform.lossyScale.y, transform.lossyScale.z);

            // Sample nodes
            samplesList = generateUniformSamplesAndComputeRadius(sampleNumber);

            // Create a Dictionary to store priority distance and next node for each agent
            nextNodeDictionary = new Dictionary<GameObject, Pair>(); 

            staticPrioDictionary = new Dictionary<string, int>(); 
            for (int i = 0; i < agentsObjects.Count; i++)
            {
                int priority = i;
                staticPrioDictionary[agentsObjects[i].name] = priority;
                Debug.Log("Agent Name: " + agentsObjects[i].name);
                Debug.Log("Static Priority:" + staticPrioDictionary[agentsObjects[i].name].ToString());
            }
            /*
             ***************************
             Setup event handlers
             ***************************
            */

            // Subscribe to agent movement event
            // StateEvents.OnAgentMoved += UpdateAgentPosition;
        }
    }

    // private void OnDestroy() {
    //     // Unsubscribe from agent movement event
    //     StateEvents.OnAgentMoved -= UpdateAgentPosition;
    // }

    // Private method for state manager to update current position of given agent
    // private void UpdateAgentPosition(int agentId, Transform newPose) {
    //     agentPositions[agentId] = newPose;
    // }

    public Dictionary<GameObject, Pair> GetNextNodeDictionary()
    {
        for (int i = 0; i < agentsObjects.Count; i++)
        {
            Pair priority = new Pair {priorityDistance = agentsObjects[i].GetComponent<RTFMT_example_multi>().priorityDistance, next_node = agentsObjects[i].GetComponent<RTFMT_example_multi>().next_node};
            nextNodeDictionary[agentsObjects[i]] = priority;
            Debug.Log("NextNodeDictionary:" + nextNodeDictionary[agentsObjects[i]].next_node);
        }

        return nextNodeDictionary;
    }

    public Dictionary<string, int> GetStaticPriority()
    {
        return staticPrioDictionary;
    }

    
    public List<Transform> GetAgents()
    {
        return new List<Transform>(agents);
    }

    public List<GameObject> GetAgentsObjects()
    {
        return agentsObjects;
    }

    public List<Transform> GetDynamicObstacles()
    {
        return new List<Transform>(dynamicObstacles);
    }

    public List<Vector3> GetSamples()
    {
        return new List<Vector3>(samplesList);
    }

    private List<Vector3> generateUniformSamplesAndComputeRadius(int numSamples)
    {
        List<Vector3> samplesList_ = new List<Vector3>(numSamples);
        int trials = 0;
        int success = 0;
        while (samplesList_.Count < numSamples)
        {
            Vector3 sampledVector = samplePoint();
            bool collision = Physics.CheckSphere(sampledVector, ballRadius, fixedObstaclesLayer);
            if (!collision)
            {
                samplesList_.Add(sampledVector);
                success++;
            }
            trials++;
        }
        computeRadius(success, trials, numSamples);
        return samplesList_;
    }

    private void computeRadius(int success, int trials, int numSampples)
    {
        float zeta = unitBallVolume(dim);
        float mu = freeVolume(success, trials);

        float gamma = 2 * Mathf.Pow((1 + 1 / (float)dim), (1 / (float)dim)) * Mathf.Pow((mu / zeta), (1 / (float)dim)); // (1 + 1 / d) is from PRM not FMT, which is 1 / d only

        rn = rnScale * gamma * Mathf.Pow((Mathf.Log10(numSampples) / (float)numSampples), (1 / (float)dim));

// #if _DEBUG_1_
		Debug.Log("The neighbor radius is: " + rn);
// #endif
    }

    // freeVolume computes the estimate of the obstacle-free part of the environment
    // using the upper confidence bound.
    // It is safer to mistake the volume bigger than it actually is.
    private float freeVolume(int success, int trials)
    {
        float area = totalArea();
        Vector2 CI = Prob.binomialInterval(success, trials);

        float freeVol = CI[1] * area;
#if _DEBUG_1_
		Debug.Log("The free volume is: " + freeVol);
#endif
Debug.Log("The free volume is: " + freeVol);

        return freeVol;
    }

    private float unitBallVolume(int dim)
    {
        Assert.IsTrue(dim >= 0);
        if (dim == 0)
        {
            return 1;
        }
        else if (dim == 1)
        {
            return 2;
        }
        else
        {
            return 2 * Mathf.PI / dim * unitBallVolume(dim - 2);

        }
    }

    private float totalArea()
    {
        float area;
        float diffx = this.xBound[1] - this.xBound[0];
        float diffy = this.yBound[1] - this.yBound[0];
        area = diffx * diffy;
        return area;
    }

    private Vector3 samplePoint()
    {
        Vector3 sampledVector = new Vector3(UnityEngine.Random.Range(xBound[0], xBound[1]), ballRadius, UnityEngine.Random.Range(yBound[0], yBound[1]));
        return sampledVector;
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
}
