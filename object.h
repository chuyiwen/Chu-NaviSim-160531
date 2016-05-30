#ifndef OBJECT
#define OBJECT
#include "define.h"

struct CAR_STAT {	// status of car(s).
    short link;		// どのリンク上にいるか？
    short pos; //そのリンク上のどの場所にいるかを，リンク原点からの距離で表現
    short speed; 	// 速度，原点から見て離れる方向だと正値．逆だと負値．停止中だとゼロ．
};

struct CAR_MOVING_HISTORY {
    bool arrived;
    int car_ID;	// 車両のID(番号)
    int birth_step;	// 当該車両が生成されたシミュレーションステップ番号
    struct CAR_STAT history[MAX_PERIOD_of_CAR_EXISTENCE];	// 当該車両の履歴
};

struct CAR_MOVING_PLAN{
    short plan_length;
    short plan[MAX_PLAN_LENGTH];
    short plan_arc[MAX_PLAN_LENGTH];
};

struct NODE{ // 道路ネットワークのノードの構造定義
    short node_ID;// ノードのID
    double x,y; // ノードの座標値（単位：メートル(m)）
    short connected_arc_num;	//本ノードに接続されるアークの数
    short connected_arc[MAX_CONNECTED_ARC_NUM];// 本ノードに接続されるアークの集合
    short connected_node[MAX_CONNECTED_ARC_NUM];// 本ノードに１本のアークを介して直接的に接続されるノードの集合
};

struct ARC{  // 道路ネットワークのアークの構造定義
    short arc_ID;// アークのID
    short ori_node,des_node;// 始点・終点のノードのID
    double length;// アークの長さ（単位：メートル(m)）

};

struct _INFO_NODE_ARC{
    struct NODE *node;
    struct ARC *arc;
    short num_node;
    short num_arc;
    short length;
};

struct _INFO_DIJKSTRA{
    int shortest_path_length;
    int shortest_distance;
    int path_plan[MAX_PLAN_LENGTH];
};

struct _INFO_CAR_MOVING{
    int num_car_moving;
    struct CAR_MOVING_HISTORY *car_moving_history;
    struct CAR_MOVING_PLAN *car_moving_plan;
};

#endif // OBJECT

