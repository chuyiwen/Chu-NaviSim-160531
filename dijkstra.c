#include "dijkstra.h"

int Adjacency[MAX_NODE_NUM][MAX_NODE_NUM];
int S[MAX_NODE_NUM];
int Length[MAX_NODE_NUM];
int Path[MAX_NODE_NUM];
int Shortest_Path[MAX_NODE_NUM];
int temp_Shortest_Path[MAX_NODE_NUM];
int shortest_path_length;
int shortest_distance;
_INFO_DIJKSTRA *adr_dijkstra;//dijkstraで最短経路のを格納する配列の先頭要素のアドレス

_INFO_DIJKSTRA *dijkstra(struct _INFO_NODE_ARC adr_node_arc, int start, int end)
{
    adr_dijkstra = (_INFO_DIJKSTRA*)malloc(sizeof(_INFO_DIJKSTRA));
    //printf("adr_dijkstra = %p\n",adr_dijkstra);

    //最短経路のを格納する配列を初期化
    adr_dijkstra->shortest_distance = 0;
    adr_dijkstra->shortest_path_length = 0;
    for(int i = 0 ; i < MAX_PLAN_LENGTH ; i++){
        adr_dijkstra->path_plan[i] = -1;
    }



    //ノードの情報を初期化する

       for(int i = 0; i < adr_node_arc.num_node ; i++){
           S[i] = 0;
           Length[i] = 999999;
           for(int j = 0 ; j < adr_node_arc.num_node ; j++){
               Adjacency[i][j] = 999999;
           }
       }
       //アークの情報を初期化する
       for(int arcID = 0; arcID < adr_node_arc.num_arc ; arcID++){
           Adjacency[(adr_node_arc.arc + arcID)->ori_node][(adr_node_arc.arc + arcID)->des_node] = (adr_node_arc.arc + arcID)->length;
           Adjacency[(adr_node_arc.arc + arcID)->des_node][(adr_node_arc.arc + arcID)->ori_node] = (adr_node_arc.arc + arcID)->length;
       }
       //Main of Dijkstra
       int size;
       int min;
       int nearest;

       S[start] = 1;
       size     = 1;

       for(int nodeID = 0 ; nodeID < adr_node_arc.num_node ; nodeID++){
           if(Adjacency[start][nodeID] >= 0){
                 Length[nodeID] = Adjacency[start][nodeID];
                 Path[nodeID]   = start;
               }
       }
       Length[start] = 0;
       Path[start]   = -1;  // -1 means start node


       while(size < (adr_node_arc.num_node)){
           min = 999999;

           for(int connect_nodeID = 0 ; connect_nodeID < adr_node_arc.num_node; connect_nodeID++){
               if(S[connect_nodeID] == 0 && min >= Length[connect_nodeID]){
                   min = Length[connect_nodeID];
                   nearest = connect_nodeID;
               }
           }
           S[nearest] = 1;
           size++;

           for(int connect_nodeID = 0 ; connect_nodeID < adr_node_arc.num_node ; connect_nodeID++){
               if(Adjacency[nearest][connect_nodeID] != 999999 && S[connect_nodeID] != 1){
                   if(Length[connect_nodeID] > (Length[nearest] + Adjacency[nearest][connect_nodeID])){
                       Length[connect_nodeID] = Length[nearest] + Adjacency[nearest][connect_nodeID];
                       Path[connect_nodeID] = nearest;
                   }
               }
           }
       }
       //-- End of Main Algorithm of Dijkstra

       int next_node;

       shortest_path_length = 0;
       next_node            = end;

       while(Path[next_node]!=-1){
           //-- "-1" represents start node.

           temp_Shortest_Path[shortest_path_length] = Path[next_node];
           shortest_path_length++;

           next_node = Path[next_node];
         }

       for(int i = 0; i < shortest_path_length; i++){
           Shortest_Path[i] = temp_Shortest_Path[shortest_path_length - (i+1)];
         }
         Shortest_Path[shortest_path_length] = end;
         shortest_path_length++;


         //-- Set "shortest_distance" based on Length[end]
         shortest_distance = Length[end];

        //save the shortest_path to sturct
        adr_dijkstra->shortest_distance = shortest_distance;
        adr_dijkstra->shortest_path_length = shortest_path_length;
        for(int i = 0 ; i < shortest_path_length ; i++){
            adr_dijkstra->path_plan[i] = Shortest_Path[i];
       //     printf("path_plan[%d] = %d\n",i,Shortest_Path[i]);
            }


    return adr_dijkstra;

}
