using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Mapper
{
    public Mapper(TerrainManager terrain_manager)
    {

    }

    public float[,] configure_obstacle_map(TerrainManager terrain_manager)
    {   
        float[,] obstacle_map = terrain_manager.myInfo.traversability;
        int xSize = obstacle_map.GetLength(0);     //5//400
        int zSize = obstacle_map.GetLength(1);      //4//400
        float xMin = terrain_manager.myInfo.x_low; 
        float xMax = terrain_manager.myInfo.x_high;
        int xNum = terrain_manager.myInfo.x_N;      //5//400
        float zMin = terrain_manager.myInfo.z_low;
        float zMax = terrain_manager.myInfo.z_high;
        int zNum = terrain_manager.myInfo.z_N;       //4//400

        float xRes = (float)Math.Ceiling((xMax - xMin)/xNum);   //40//0.5
        float zRes = (float)Math.Ceiling((zMax - zMin)/zNum);   //50//0.5

        float[,] intermediate_map1 = new float[(int)(xSize*xRes), zNum];  //5*40 *  4=800//200 *400
        float[,] intermediate_map2 = new float[(int)(xSize*xRes), (int)(zSize*zRes)]; //5*40 * 4*50=40000  // 200* 200
        float[,] intermediate_map3 = new float[(int)(xSize*xRes), (int)(zSize*zRes)];  //40000 
        float[,] new_obstacle_map = new float[(int)(xSize*xRes), (int)(zSize*zRes)];  //40000

        if (xRes >= 1)
        {
            for (int i = 0; i < xNum; i++)
            {
                for (int j = 0; j < xRes; j++)
                {
                    for (int k = 0; k < zNum; k++)
                    {
                        float val = obstacle_map[i, k];
                        intermediate_map1[j + (int)xRes * i, k] = val;
                    }
                }
            }
        }
        else
        {
            for (int i = 0; i < xNum; i++)
            {
                for (int k = 0; k < zNum; k++)
                {
                    float val = obstacle_map[i, k];
                    intermediate_map1[(int)Math.Floor(xRes) * i, k] = val;
                }
            }
        }
        if(zRes >= 1)
        {
            for (int i = 0; i < zNum; i++)
            {
                for (int j = 0; j < zRes; j++)
                {
                    for (int k = 0; k < (int)(xSize * xRes); k++)
                    {
                        float val = intermediate_map1[k, i];
                        intermediate_map2[k, j + (int)zRes * i] = val;
                    }
                }
            }
        }
        else
        {
            for (int i = 0; i < zNum; i++)
            {
                for (int k = 0; k < (int)(xSize * xRes); k++)
                {
                    float val = intermediate_map1[k, i];
                    intermediate_map2[k, (int)Math.Floor(zRes) * i] = val;
                }
            }
        }
            

        // pading the obstacle  
        for (int i = 1; i < (int)(xSize * xRes) - 1; i++)
        {
            for (int j = 1; j < (int)(zSize * zRes) - 1; j++)
            {
                float neighbours = intermediate_map2[i - 1, j] + intermediate_map2[i - 1, j - 1] + intermediate_map2[i, j - 1] + intermediate_map2[i + 1, j - 1] + intermediate_map2[i + 1, j] + intermediate_map2[i + 1, j + 1] + intermediate_map2[i, j + 1] + intermediate_map2[i - 1, j + 1];
                if (neighbours >= 1)
                {
                    intermediate_map3[i, j] = 1;
                }
            }
        }

        for (int i = 1; i < (int)(xSize * xRes) - 1; i++)
        {
            for (int j = 1; j < (int)(zSize * zRes) - 1; j++)
            {
                float neighbours = intermediate_map3[i - 1, j] + intermediate_map3[i - 1, j - 1] + intermediate_map3[i, j - 1] + intermediate_map3[i + 1, j - 1] + intermediate_map3[i + 1, j] + intermediate_map3[i + 1, j + 1] + intermediate_map3[i, j + 1] + intermediate_map3[i - 1, j + 1];
                if (neighbours >= 1)
                {
                    new_obstacle_map[i, j] = 1;
                }
            }
        }

        return new_obstacle_map;
    }
}