using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    [RequireComponent(typeof(DroneController))]
    public class PurePursuit 
    {
        public float view = 2;                 //look-ahead distance
        public float viewcontrol = 0.1f;       //look forward gain
        public float speedcontrol = 1;         //speed proportional gain
        public float timeslot = 0.1f;          // time tick
        public float axlesize = 2.9f;          //wheel base of vehicle

        public float velocity;

        public int ind;

        Node current;
        List<Vector3> my_path = new List<Vector3>();
        List<Node> track_Node = new List<Node>();

        public PurePursuit(Node current, List<Vector3> my_path, float velocity)
        {
            this.current = current;
            this.my_path = my_path;
            this.velocity = velocity;
        }

        public List<Node> PurePursuitA()
        {

            //speed (m/s)
            float tager_speed = 10 / 3.6f;
            float time = 500;  # max simulation time

            ind = calc_Target_Index(current, my_path);

            while (time!= 0 && ind< my_path.Count)
            {
                //Calculate control input
                float ai = Pcontrol(tager_speed, velocity);
                float di = PurePursuitControl(current, my_path, ind);

                current = update(current, ai, di);
                track_Node.Add(current);
                
                time = time -timeslot;
            }
            return track_Node;
        }

        public Node update(Node current,float a,float delta)
        {
            current.x = current.x + velocity * (float)Math.Cos(current.theta) * timeslot;
            current.z = current.z + velocity * (float)Math.Sin(current.theta) * timeslot;
            current.theta = current.theta + velocity / axlesize * (float)Math.Tan(delta) * timeslot;
            velocity = velocity + a * timeslot;
            return current; 
        }
        public float Pcontrol(float target,float current)
        {
            float distance = speedcontrol * (target - current);

            return distance;
        }
        public float  PurePursuitControl(Node current, List<Vector3> my_path, int pind)
        {
            ind = calc_Target_Index(current, my_path);

            if (pind >= ind)
                ind = pind;

            float tx = 0;
            float tz = 0;
            if(ind< my_path.Count)
            {
                tx = my_path[ind].x;
                tz = my_path[ind].z;

            }
            else  //# toward goal
            {
                ind = my_path.Count - 1;
            }
            float alpha = (float)Math.Atan2(tz - current.z, tx - current.x) - current.theta;
            if (velocity < 0)
            {
                alpha = (float)Math.PI - alpha;

            }
            //# update look ahead distance
            float Lf = viewcontrol * velocity + view;

            float delta = (float)Math.Atan2(2.0 * axlesize * Math.Sin(alpha) / Lf, 1.0);

            return delta;
        }
        private float calculateEuclidean(float x1, float z1, float x2, float z2)
        {
            return (float)Math.Sqrt(Math.Pow((x1 - x2), 2) + Math.Pow((z1 - z2), 2));
        }
        private int calc_Target_Index(Node current, List<Vector3> my_path)
        {
            List<float> dx = new List<float>();
            List<float> dy = new List<float>();
            List<float> dist = new List<float>();
            int ind = 0;
            float store = 1000;
            foreach (Vector3 one in my_path)
            {
                dx.Add(current.x- one.x);
                dy.Add(current.z- one.z);
                dist.Add(calculateEuclidean(current.x, current.z, one.x, one.z));
            }
            for(int i =0;i<dist.Count;i++)
            {
                if(dist[i]<store)
                {
                    store = dist[i];
                    ind = i;
                }
            }
            float L = 0;

            //# update look ahead distance
            float Lf = viewcontrol * velocity + view;

            while(Lf>L && (ind+1)< dist.Count)
            {
                L += (float)Math.Sqrt(Math.Pow(my_path[ind+1].x- my_path[ind].x, 2.0)+ Math.Pow(my_path[ind + 1].z - my_path[ind].z, 2.0));
                ind++;
            }

            return ind; 
        }
    }
}