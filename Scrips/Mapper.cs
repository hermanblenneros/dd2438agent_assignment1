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
        float xMin = terrain_manager.myInfo.x_low;
        float xMax = terrain_manager.myInfo.x_high;
        int xNum = terrain_manager.myInfo.x_N;
        float zMin = terrain_manager.myInfo.z_low;
        float zMax = terrain_manager.myInfo.z_high;
        int zNum = terrain_manager.myInfo.z_N;

        float xRes = (xMax - xMin)/xNum;
        float zRes = (zMax - zMin)/zNum;

        float[,] intermediate_map1 = new float[(int)(xMax-xMin), zNum];
        float[,] intermediate_map2 = new float[(int)(xMax-xMin), (int)(zMax-zMin)];
        float[,] new_obstacle_map = new float[(int)(xMax-xMin), (int)(zMax-zMin)];

        for(int i = 0; i < xNum; i++)
        {
            for(int j = 0; j < xRes; j++)
            {
                for(int k = 0; k < zNum; k++)
                {
                    float val = obstacle_map[i,k];
                    intermediate_map1[j + (int)xRes*i, k] = val;
                }
            }
        }

        for(int i = 0; i < zNum; i++)
        {
            for(int j = 0; j < zRes; j++)
            {
                for(int k = 0; k < (int)(xMax-xMin); k++)
                {
                    float val = intermediate_map1[k,i];
                    intermediate_map2[k,j + (int)zRes*i] = val;
                }
            }
        }

        for(int i = 1; i < (int)(xMax-xMin) - 1; i++)
        {
            for(int j = 1; j < (int)(xMax-xMin) - 1; j++)
            {
                float neighbours = intermediate_map2[i-1,j] + intermediate_map2[i-1,j-1] + intermediate_map2[i, j-1] + intermediate_map2[i+1,j-1] + intermediate_map2[i+1,j] + intermediate_map2[i+1,j+1] + intermediate_map2[i,j+1] + intermediate_map2[i-1, j+1];
                if(neighbours >= 1)
                {
                    new_obstacle_map[i,j] = 1;
                }
            }
        }
        return new_obstacle_map;
    }
}