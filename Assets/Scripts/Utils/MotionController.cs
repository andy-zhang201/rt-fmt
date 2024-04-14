using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MotionController{

    public float gain = 2.0f;
    public float default_gain = 2.0f;
    RTFMT_example_multi unityComponent;
    float executedCost;
    Vector3 lastPos;

    // PARAM: How close agents need to be before velocity planning kicks in
    float closeness_threshold = 20.0f; //High value for testing purposes

    // public MotionController(RTFMT_example_multi rtfmt, float gain)
    // {
    //     this.gain = gain;
    //     this.unityComponent = rtfmt;
    //     this.lastPos = this.unityComponent.transform.position;
    // }

    public MotionController(MonoBehaviour rtfmt, float gain)
    {
        this.gain = gain;
        this.unityComponent = (RTFMT_example_multi)rtfmt;
        this.lastPos = this.unityComponent.transform.position;
    }

    public void setGain(float gain)
    {
        this.gain = gain;
    }

    public void control(Vector3 setpoint, float clearance)
    {
        // how to change velocity:
        /*
        1. Access to individual velocity
        2. Access to individual priority AND other agent's priority
        3. Calculate distance between individual and other agents
        4. Calculate closeness to goal and compare with other agent's closeness to goal
        */

        // Calculate error
        Vector3 error = (setpoint - this.unityComponent.transform.position);

        // Reset gain
        this.gain = this.default_gain;

        // Get transforms of other agents from state manager
        List<Transform> tf_other_agents = StateManager.instance.GetAgents();
        tf_other_agents.Remove(this.unityComponent.transform);

        // Get static priority dictionary from state manager
        Dictionary<string, int> staticPriorityDict = StateManager.instance.GetStaticPriority();
        
        // Get this agent priority
        GameObject agentObj = this.unityComponent.gameObject;
        int agentPriority = staticPriorityDict[agentObj.name];

        float goalCloseness = Vector3.Distance(this.unityComponent.transform.position, this.unityComponent.goalPosition);
        
        Debug.Log("--------------------");
        Debug.Log("Agent Name: "+ agentObj);
        Debug.Log("Agent Velocity: "+ this.unityComponent.GetComponent<Rigidbody>().velocity);
        Debug.Log("Agent Priority: " + agentPriority);
        Debug.Log("Agent Closeness to Goal: " + goalCloseness);
        Debug.Log("Other Agent name 1: " + tf_other_agents[0].gameObject.name + "Other Agent1 Priority: "+ staticPriorityDict[tf_other_agents[0].gameObject.name]);
        Debug.Log("Other Agent name 2: " + tf_other_agents[1].gameObject.name + "Other Agent2 Priority: "+ staticPriorityDict[tf_other_agents[1].gameObject.name]);
        Debug.Log("--------------------");
        
        float slowFactor = 1.0f; // reduce this for every higher priority agent close by that's closer to the goal than the current agent.
        

        foreach(Transform tf in tf_other_agents)
        {
            // For every other agent, check if it's close by.
            float dist = Vector3.Distance(this.unityComponent.transform.position, tf.position);
            if(dist < closeness_threshold)
            {
                // If it is close by, check if it's closer to the goal than the current agent and if it's higher priority
                // Find closeness other agent's closeness to goal
                float otherAgentCloseness = Vector3.Distance(tf.position, this.unityComponent.goalPosition);

                // Find priority of other agent
                string agentName = tf.gameObject.name;
                int otherAgentPriority = staticPriorityDict[agentName];
                
                if((otherAgentCloseness < goalCloseness) & (otherAgentPriority > agentPriority))
                {
                    // Reduce slowfactor
                    slowFactor *= 0.75f;
                }
            }
        }

        // Reduce gain by slowFactor
        this.gain *= slowFactor;

        // Debug.Log("Setpoint: " + setpoint+ ", Vel vector: " + velocityVector +  ", Object position: " + this.unityComponent.transform.position);
        // Debug.Log(velocityVector.magnitude);

        Vector3 velocityVector = this.gain * error.normalized;
        Debug.Log("Final Velocity: " + velocityVector.magnitude);

        if(error.magnitude > clearance)
        {
            this.unityComponent.GetComponent<Rigidbody>().velocity = velocityVector;
        }
        else
        {
            this.unityComponent.GetComponent<Rigidbody>().velocity = Vector3.zero;
            this.unityComponent.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;

        }

        updateExecutedCost();
    }

    public void clearCost()
    {
        this.lastPos = this.unityComponent.transform.position;
        this.executedCost = 0;
    }
    public float getExecutedCost()
    {
        return this.executedCost;
    }
    public void updateExecutedCost()
    {
        Vector3 pos = this.unityComponent.transform.position;
        Vector3 dPos = pos - this.lastPos;
        
        float dist = dPos.magnitude;
        this.executedCost += dist;

        this.lastPos = pos;
    }

}
