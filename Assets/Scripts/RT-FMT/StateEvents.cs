using UnityEngine;

public static class StateEvents {
    /*
     ************************* 
     * Agent movement event
     *************************
    */
    public delegate void AgentMovementEventHandler(int agentId, Vector3 newPos);
    public static event  AgentMovementEventHandler OnAgentMoved;

    // Call agent movement method
    public static void AgentMoved(int agentId, Vector3 newPos) {
        OnAgentMoved?.Invoke(agentId, newPos);
    }

    /*
     ********************************** 
     * Dynamic obstacle movement event
     **********************************
    */
}