#include "car.h"


CAR_MOVING_HISTORY *car_moving_history;
CAR_MOVING_PLAN *car_moving_plan;
_INFO_DIJKSTRA *_info_dijkstra;
_INFO_CAR_MOVING *_info_car_moving;

int car_step[MAX_CAR_NUM];



void *add_car_move_plan(struct _INFO_NODE_ARC adr_node_arc,struct _INFO_CAR_MOVING _info_car_moving,int init_start_carID,int init_car_num,int sim_step){
     
     	_info_dijkstra = (_INFO_DIJKSTRA*)malloc(sizeof(_INFO_DIJKSTRA));

    //経路情報を書き込み    ----    OK
    for(int i = 0 ; i < init_car_num ; i++){
	(_info_car_moving.car_moving_history+init_start_carID+i)->birth_step = sim_step;
        _info_dijkstra = dijkstra(adr_node_arc,99,0);
        (_info_car_moving.car_moving_plan+init_start_carID+i)->plan_length = _info_dijkstra->shortest_path_length;
        for(int j = 0 ; j < (_info_car_moving.car_moving_plan+init_start_carID+i)->plan_length ; j++){
        (_info_car_moving.car_moving_plan+init_start_carID+i)->plan[j] = _info_dijkstra->path_plan[j];
//	printf("car_moving_plan %d plan[%d] = %d\n",i,j,(_info_car_moving.car_moving_plan+init_start_carID+i)->plan[j]);    //-----node
        }
        for(int k = 0 ; k < ((_info_car_moving.car_moving_plan+init_start_carID+i)->plan_length - 1) ; k++){
      	    for(int n = 0 ; n < (adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID+i)->plan[k])->connected_arc_num;n++){
          	  if((adr_node_arc.arc+((adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID+i)->plan[k])->connected_arc[n]))->des_node == 	(_info_car_moving.car_moving_plan+init_start_carID+i)->plan[k+1])
          	  (_info_car_moving.car_moving_plan+init_start_carID+i)->plan_arc[k] = (adr_node_arc.node+(_info_car_moving.car_moving_plan+init_start_carID+i)->plan[k])->connected_arc[n];

   		     }
        }
    }

	//導入した位置を修正する(導入した前には-1です)
	for(int i = 0 ; i < init_car_num ; i++){
	(_info_car_moving.car_moving_history+init_start_carID+i)->history[0].link = (_info_car_moving.car_moving_plan+init_start_carID+i)->plan_arc[0];
	(_info_car_moving.car_moving_history+init_start_carID+i)->history[0].pos = 0;
	}

    //経路を表示
//    for(int i = 0 ; i < init_car_num ; i++){
//        printf("Car %d have %d steps to end,",init_start_carID+i,(_info_car_moving.car_moving_plan+init_start_carID+i)->plan_length);
//        for(int j = 0 ; j < (_info_car_moving.car_moving_plan+init_start_carID+i)->plan_length+1; j++)
//            printf("%d ",(_info_car_moving.car_moving_plan+init_start_carID+i)->plan[j]);
//        printf("\n");
//    }

	//経路情報のメモリをfree
	free(_info_dijkstra);
 }


void car_move(struct _INFO_NODE_ARC adr_node_arc)
{

	car_moving_history = (CAR_MOVING_HISTORY*)malloc(MAX_CAR_NUM*sizeof(CAR_MOVING_HISTORY));
     	car_moving_plan =(CAR_MOVING_PLAN*)malloc(MAX_CAR_NUM*sizeof(CAR_MOVING_PLAN));
     	_info_car_moving = (_INFO_CAR_MOVING*)malloc(sizeof(_INFO_CAR_MOVING));

	_info_car_moving->num_car_moving = 0;
	_info_car_moving->car_moving_history = car_moving_history;
	_info_car_moving->car_moving_plan = car_moving_plan;

	//初期化
	int sim_step = 0;
        //CAR_MOVING_HISTORYを初期化   ---   OK
    	 for(int i = 0 ; i < MAX_CAR_NUM ; i++){
		  car_step[i] = 0 ;
        	 (car_moving_history+i)->arrived = false;
		 (car_moving_history+i)->car_ID = i ;
        	 (car_moving_history+i)->birth_step = -1; //-1 means not exist
         	for(int j = 0 ; j < MAX_PLAN_LENGTH;j++){
         	    (car_moving_history+i)->history[j].link = -1;   //-1 means not exist
         	    (car_moving_history+i)->history[j].pos = -1;
         	    (car_moving_history+i)->history[j].speed=10;
         	}
     	 }


       //CAR_MOVING_PLANを初期化    ---   OK
 	 for(int i = 0 ; i < MAX_CAR_NUM ; i++){
         (car_moving_plan+i)->plan_length = 0;
         	for(int j = 0 ; j <MAX_PLAN_LENGTH;j++){
                (car_moving_plan+i)->plan[j] = -1;
         	(car_moving_plan+i)->plan_arc[j] = -1;
    		}
     	 }



	printf("初期化完了\n");

	

	//main part of moving
	while(sim_step <= 100){
		
		//Add cars to simulation
	      if(_info_car_moving->num_car_moving+10 <= MAX_CAR_NUM){		
		if(sim_step % 5 == 0){
		//printf("Added cars in %d steps\n",sim_step);
		add_car_move_plan(adr_node_arc,*_info_car_moving,_info_car_moving->num_car_moving,10,sim_step);
		_info_car_moving->num_car_moving+=10;
		}
	       }

		//Dijkstraで計算した経路で移動する
		for(int i = 0 ; i < _info_car_moving->num_car_moving ; i++){
          	  if(!(car_moving_history+i)->arrived){
         	   (car_moving_history+i)->history[sim_step - ((car_moving_history+i)->birth_step)].link = (car_moving_plan+i)->plan_arc[car_step[i]];
         	   (car_moving_history+i)->history[sim_step - ((car_moving_history+i)->birth_step)+1].pos = (car_moving_history+i)->history[sim_step - ((car_moving_history+i)->birth_step)].pos+(car_moving_history+i)->history[sim_step - ((car_moving_history+i)->birth_step)].speed;
          	  }
      	        }

		
		//到着を判断する
      	       for(int i = 0 ; i < _info_car_moving->num_car_moving ; i++){
            	 if(!(car_moving_history+i)->arrived){
              	   if((car_step[i] >= ((car_moving_plan+i)->plan_length - 1))){
                	(car_moving_history+i)->arrived = true;
              	//	printf("Car %d was arried in %d steps.\n",i,sim_step);
              	   }
            	 }
               }


		//次のアークに移動したを判断する
    	       for(int i = 0 ; i < _info_car_moving->num_car_moving ; i++){
		    if(!(car_moving_history+i)->arrived){
			int move_step = sim_step - ((car_moving_history+i)->birth_step);
			//printf("move_step = %d \n",move_step);
                   	if((car_moving_history+i)->history[move_step+1].pos >= (adr_node_arc.arc+(car_moving_history+i)->history[move_step].link)->length){
			int now_pos = (car_moving_history+i)->history[move_step].pos;
			int now_link = (car_moving_history+i)->history[move_step].link;
			(car_moving_history+i)->history[move_step+1].pos -= (adr_node_arc.arc+now_link)->length;
                	car_step[i]++;           
              		}
            	   }
        	}

		//履歴を表示  --  OK
		for(int i = 0 ; i < _info_car_moving->num_car_moving ; i++){
			if((car_moving_history+i)->arrived)
				break;
		printf("Car %d in link %d pos %d at %d steps\n",i,(car_moving_history+i)->history[sim_step].link,(car_moving_history+i)->history[sim_step].pos,sim_step);
		}
		
	        
		sim_step++;
		printf("step = %d\n",sim_step);	    
	}	 
	printf("Done!\n");
}




       

        






