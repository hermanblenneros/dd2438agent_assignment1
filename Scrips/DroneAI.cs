using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    //// Declaration of variables

    // Unity variables
    private DroneController m_Drone;
    Rigidbody my_rigidbody;
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    // The start position
    Vector3 start_pos;
    Vector3 goal_pos;
    List<Node> my_path = new List<Node>();

    // Tracking variables
    public float k_p = 2f, k_d = 0.5f;
    float to_path, to_target, distance, steering, acceleration, starting_timer = 0, stuck_timer = 0, reverse_timer = 0, break_timer = 0, old_acceleration = 0, new_acceleration, acceleration_change, my_speed = 0, old_angle, new_angle, angle_change, unstuck_error, old_unstuck_error = 100, unstuck_error_change;
    int to_path_idx, to_target_idx, dummy_idx, dummy_idx2, lookahead = 0, my_max_speed = 25;
    bool starting_phase = true, is_stuck = false, is_breaking = false, counting = false, no_waypoint = true;
    Vector3 pos, difference, target_position, aheadOfTarget_pos, target_velocity, position_error, velocity_error, desired_acceleration, closest, null_vector = new Vector3(0,0,0);
    Node target, aheadOfTarget, closestNode;
    List<float> controls = new List<float>(), cum_angle_change = new List<float>();

    //// Definition of functions

    // Function that computes the appropriate lookahead depending on the current speed
    public int speedToLookahead(float my_speed)
    {
        lookahead = (int)(4 + Math.Sqrt(my_speed));
        return lookahead;
    }
    
    private void Start()
    {
        // Get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
        my_rigidbody = GetComponent<Rigidbody>();
        start_pos = terrain_manager.myInfo.start_pos;
        goal_pos = terrain_manager.myInfo.goal_pos;

        // Create mapper and compute obstacle map
        Debug.Log("Creating obstacle map of current terrain");
        Mapper mapper = new Mapper(terrain_manager);
        float[,] obstacle_map = mapper.configure_obstacle_map(terrain_manager);
        float[,] distance_map = mapper.configure_distance_map(obstacle_map);

        // Create planner and find path
        Debug.Log("Planning path");
        Planner planner = new Planner();
        Node goalNode = planner.AStar(terrain_manager, m_Drone, start_pos, goal_pos, obstacle_map, distance_map, 10000);

        if (goalNode == null)
        {
            Debug.Log("Pathing failed");
            this.enabled = false;
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

        // Removing abudant nodes from path
        DouglasPeucker dp = new DouglasPeucker();
        List<Node> dp_path = dp.DouglasPeuckerReduction(my_path, 1);
        Vector3 old_wpp = start_pos;
        foreach (Node n in dp_path)
        {
            Vector3 wp = new Vector3(n.x, 0, n.z);
            Debug.DrawLine(old_wpp, wp, Color.green, 100f);
            old_wpp = wp;
        }
    }

    private void FixedUpdate()
    {
        // Execute your path here
        
        // Decide on lookahead based on speed;
        lookahead = speedToLookahead(my_speed);

        // Finding closest node on path
        to_path = 100;
        to_path_idx = 0;
        dummy_idx = 0;
        foreach(Node node in my_path)
        {
            pos = new Vector3(node.x, 0, node.z);
            difference = pos - transform.position;
            distance = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );
                    
            if(distance < to_path)
            {
                to_path = distance;
                to_path_idx = dummy_idx;
            }
            dummy_idx += 1;
        }

        // Saving data about node on path closest to the car
        closestNode = my_path[to_path_idx];
        closest = new Vector3(closestNode.x, 0, closestNode.z);
        difference = closest - transform.position;
        to_path = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

        // Finding target node on path
        to_target = 100;
        to_target_idx = 0;
        dummy_idx2 = 0;
        foreach(Node node in my_path)
        {
            if(dummy_idx2 < to_path_idx)
            {   
                dummy_idx2 += 1;
                continue;
            }

            pos = new Vector3(node.x, 0, node.z);
            difference = pos - closest;
            distance = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

            if(Math.Abs(distance + to_path - lookahead) < to_target)
            {
                to_target = Math.Abs(distance + to_path - lookahead);
                to_target_idx = dummy_idx2;
            }
            dummy_idx2 += 1;
        }

        // Break condition
        try
        {
            target = my_path[to_target_idx];
            aheadOfTarget = my_path[to_target_idx + 1];
        }
        catch(ArgumentOutOfRangeException e)
        {   
            Debug.Log("Goal reached");
            this.enabled = false;
        }

        // Keep track of target position and velocity
        target_position = new Vector3(target.x, 0, target.z);
        aheadOfTarget_pos = new Vector3(aheadOfTarget.x, 0, aheadOfTarget.z);
        target_velocity = aheadOfTarget_pos-target_position;
        Debug.DrawLine(transform.position, target_position);
        Debug.DrawLine(transform.position, transform.position + 10*target_velocity, Color.black);

        // a PD-controller to get desired velocity
        position_error = target_position - transform.position;
        velocity_error = target_velocity.normalized - m_Drone.velocity.normalized;
        desired_acceleration = k_p * position_error.normalized + k_d * velocity_error.normalized;
        Debug.DrawLine(transform.position, transform.position + desired_acceleration);
        // Apply controls
        steering = Vector3.Dot(desired_acceleration, transform.right);
        acceleration = Vector3.Dot(desired_acceleration, transform.forward);
        m_Drone.Move(desired_acceleration.x, desired_acceleration.z);

    }

 

    // Update is called once per frame
    void Update()
    {
        
    }
}
