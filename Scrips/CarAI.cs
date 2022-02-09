﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {   
        //// Declaration of variables
        // Unity variables
        private CarController m_Car;
        Rigidbody my_rigidbody;
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        // Planning variables
        Vector3 start_pos;
        Vector3 goal_pos;
        List<Node> my_path = new List<Node>();
        // Tracking variables
        public float k_p = 2f, k_d = 0.5f;
        float minDist, minDist2, dist, steering, acceleration, timer = 0, reverse_timer = 0, break_timer = 0, old_acc_amp = 0, my_speed, to_path, old_angle, new_angle, angle_change;
        int minDistIdx, minDistIdx2, Idx, Idx2, lookahead = 0, my_topSpeed;
        bool is_stuck = false, starting_out = true, is_breaking = false, counting = false;
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
        // Function that computes the appropriate maximum speed depending on the curvature ahead
        public int curvatureToSpeed(float curvature_ahead)
        {
            int multiple = (int)Math.Round( (Math.Abs( curvature_ahead / (Math.PI/4) )) );

            if(multiple > 2)
            {
                return (int)5;
            }
            else
            {
                return (int)25;
            }
        }

        private void Start()
        {
            // Get the car controller
            m_Car = GetComponent<CarController>();

            // Get the terrain manager
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // Get the rigid body
            my_rigidbody = GetComponent<Rigidbody>();

            // Get start and goal position
            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            
            // Create mapper and compute obstacle map
            Mapper mapper = new Mapper(terrain_manager);
            float[,] obstacle_map = mapper.configure_obstacle_map(terrain_manager);

            // Create planner and find path
            Debug.Log("Planning path");
            Planner planner = new Planner();
            Node goalNode = planner.HybridAStar(terrain_manager, m_Car, start_pos, (float)Math.PI/2, goal_pos, obstacle_map, 10000);
            
            // Disable code if path not found
            if(goalNode == null)
            {
                Debug.Log("Pathing failed");
                this.enabled = false;
            }

            // Construct path
            my_path.Add(goalNode);
            Node parent = goalNode.parent;
            while(parent != null)
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

            DouglasPeucker dp = new DouglasPeucker();
            List<Node> dp_path = dp.DouglasPeuckerReduction(my_path, 1);
            Vector3 old_wpp = start_pos;
            foreach (Node n in dp_path)
            {
                Vector3 wp = new Vector3(n.x, 0, n.z);
                Debug.DrawLine(old_wpp, wp, Color.green, 100f);
                old_wpp = wp;
            }

            // Getting controls of path
            for (int i = 0; i < my_path.Count; i++)
            {   
                if(i == 0)
                {
                    old_angle = my_path[i].theta;
                    controls.Add(0);
                }

                new_angle = my_path[i].theta;
                angle_change = new_angle - old_angle;
                old_angle = new_angle;

                controls.Add(angle_change);
            }

            // Approximating cumulative forward angle changes in path
            for(int i = 0; i < my_path.Count - 11; i++)
            {
                cum_angle_change.Add(controls[i] + controls[i+1] + controls[i+2] + controls[i+3] + controls[i+4] + controls[i+5] + controls[i+6] + controls[i+7] + controls[i+8] + controls[i+9] + controls[i+10] + controls[i+10]);
            }
            
            // Going to FixedUpdate()
            Debug.Log("Tracking path");
        }

        private void FixedUpdate()
        {
            // Tracks the path generated by planner
            // Decide on lookahead
            lookahead = speedToLookahead(my_speed);

            // Tracks forward along the path if not stuck
            if(!is_stuck)
            {   
                // Finding closest node on path
                minDist = 100;
                minDistIdx = 0;
                Idx = 0;
                foreach(Node node in my_path)
                {
                    pos = new Vector3(node.x, 0, node.z);
                    difference = pos - transform.position;
                    dist = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );
                    
                    if(dist < minDist)
                    {
                        minDist = dist;
                        minDistIdx = Idx;
                    }
                    Idx += 1;
                }

                // Saving data about closest node
                closestNode = my_path[minDistIdx];
                closest = new Vector3(closestNode.x, 0, closestNode.z);
                difference = closest - transform.position;
                to_path = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                // Finding target node on path
                minDist2 = 100;
                minDistIdx2 = 0;
                Idx2 = 0;
                foreach(Node node in my_path)
                {
                    if(Idx2 < minDistIdx)
                    {   
                        Idx2 += 1;
                        continue;
                    }

                    pos = new Vector3(node.x, 0, node.z);
                    difference = pos - closest;
                    dist = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                    if(Math.Abs(dist + to_path - lookahead) < minDist2)
                    {
                        minDist2 = Math.Abs(dist + to_path - lookahead);
                        minDistIdx2 = Idx2;
                    }
                    Idx2 += 1;
                }

                // Break condition
                try
                {
                    target = my_path[minDistIdx2];
                    aheadOfTarget = my_path[minDistIdx2 + 1];
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
                velocity_error = target_velocity - my_rigidbody.velocity;
                desired_acceleration = k_p * position_error + k_d * velocity_error;

                // Apply controls
                float steering = Vector3.Dot(desired_acceleration, transform.right);
                float acceleration = Vector3.Dot(desired_acceleration, transform.forward);
                
                if(!is_breaking)
                {
                    my_topSpeed = curvatureToSpeed(cum_angle_change[minDistIdx2]);
                    Debug.Log("Maximum speed: " + my_topSpeed);
                }

                if(my_speed > my_topSpeed || is_breaking)
                {   
                    is_breaking = true;
                    m_Car.Move(steering, -acceleration, -acceleration, 0f);

                    if(my_speed <= my_topSpeed)
                    {   
                        if(break_timer <= Time.time && !counting)
                        {
                            break_timer = Time.time + 5;
                            counting = true;
                            m_Car.Move(steering, acceleration, acceleration, 0f);
                        }
                        
                        if(Time.time > break_timer)
                        {
                            counting = false;
                            is_breaking = false;
                            m_Car.Move(steering, acceleration, acceleration, 0f);
                        }
                    }
                }
                else
                {
                    m_Car.Move(steering, acceleration, acceleration, 0f);
                }

                // State variables for stuck condition
                my_speed = (float) Math.Sqrt( Math.Pow(my_rigidbody.velocity.x, 2) + Math.Pow(my_rigidbody.velocity.z, 2) );
                //Debug.Log("My speed: " + my_speed);
                float acc_amp = (float) Math.Sqrt( Math.Pow(desired_acceleration.x, 2) + Math.Pow(desired_acceleration.z, 2) );
                float acc_change = acc_amp - old_acc_amp;
                old_acc_amp = acc_amp;

                // Check stuckness. If stuck -> go to else statement below
                if(my_speed < 0.3 && acc_change < 0.0001)
                {   
                    if(timer <= Time.time && !counting)
                    {   
                        counting = true;
                        timer = Time.time + 5;
                    }

                    if(Time.time > timer)
                    {   
                        Debug.Log("Car stuck detected");
                        counting = false;
                        is_stuck = true;
                    }
                }
            }
            else // If stuck criterion == true
            {   
                if(reverse_timer <= Time.time && !counting)
                {   
                    counting = true;
                    Debug.Log("Getting unstuck");
                    reverse_timer = Time.time + 1;
                }

                if(Time.time > reverse_timer)
                {   
                    counting = false;
                    is_stuck = false;
                }

                // Finding closest node on path
                minDist = 100;
                minDistIdx = 0;
                Idx = 0;
                foreach(Node node in my_path)
                {
                    pos = new Vector3(node.x, 0, node.z);
                    difference = transform.position - pos;
                    dist = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );
                    if(dist < minDist)
                    {
                        minDist = dist;
                        minDistIdx = Idx;
                    }

                    Idx += 1;
                }

                // Decide on "lookbehind"
                lookahead = 4;

                // Break condition
                try
                {
                    target = my_path[minDistIdx - lookahead];
                }
                catch(ArgumentOutOfRangeException e)
                {   
                    ;
                }

                // Keep track of target position and velocity
                target_position = new Vector3(target.x, 0, target.z);
                target_velocity = null_vector;

                // a PD-controller to get desired velocity
                position_error = target_position - transform.position;
                velocity_error = target_velocity - my_rigidbody.velocity;
                desired_acceleration = k_p * position_error + k_d * velocity_error;

                // Apply controls
                steering = Vector3.Dot(desired_acceleration, transform.right);
                acceleration = Vector3.Dot(desired_acceleration, transform.forward);
                m_Car.Move(steering, acceleration, acceleration, 0f);

            }
        }
    }
}
