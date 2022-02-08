using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    // The drone controller
    private DroneController m_Drone;

    // The terrain manager object
    public GameObject terrain_manager_game_object;

    // The terrain manager
    TerrainManager terrain_manager;

    // The start position
    Vector3 start_pos;

    // The goal position
    Vector3 goal_pos;

    // Our planned path
    List<Node> my_path = new List<Node>();

    private void Start()
    {
        // Get the drone controller
        m_Drone = GetComponent<DroneController>();
        // Get the terrain manager
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

        // Getting start and goal position
        start_pos = terrain_manager.myInfo.start_pos;
        goal_pos = terrain_manager.myInfo.goal_pos;

        // Create mapper and compute obstacle map
        Mapper mapper = new Mapper(terrain_manager);
        float[,] obstacle_map = mapper.configure_obstacle_map(terrain_manager);

        // Create planner and find path
        Planner planner = new Planner();
        Debug.Log("Planning path");
        Node goalNode = planner.AStar(terrain_manager, m_Drone, start_pos, goal_pos, obstacle_map, 10000);

        if (goalNode != null)
        {
            Debug.Log("Path found");
        }
        else
        {
            Debug.Log("Path failed");
        }

        // Construct path
        my_path.Add(goalNode);
        Node parent = goalNode.parent;
        while (parent != null)
        {
            my_path.Add(parent);
            parent = parent.parent;
        }

        my_path.Reverse();

        // Plot path
        Vector3 old_wp = start_pos;
        foreach (Node n in my_path)
        {
            Vector3 wp = new Vector3(n.x, 0, n.z);
            Debug.DrawLine(old_wp, wp, Color.red, 100f);
            old_wp = wp;
        }
        Debug.Log("Tracking path");
    }

    private void FixedUpdate()
    {
        // Execute your path here
        // ...

      
        // this is how you control the car
        //m_Drone.Move(0.4f * Mathf.Sin(Time.time * 1.9f), 0.1f);

    }

 

    // Update is called once per frame
    void Update()
    {
        
    }
}
