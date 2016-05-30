#include "gen_graph.h"

NODE *node_array;//ノードを格納する配列の先頭要素のアドレス
ARC *arc_array;//アークを格納する配列の先頭要素のアドレス
_INFO_NODE_ARC *adr_node_arc;//nodeとarcの情報を格納する配列の先頭要素のアドレス

_INFO_NODE_ARC *gen_lattice(int n, int length) //格子状計算関数
{
    node_array = (NODE*)malloc((n*n)*sizeof(NODE));//ノードのメモリを定義
  //  printf("%p\n",node_array);
    arc_array  = (ARC*)malloc((4*n*(n-1))*sizeof(ARC));//アークのメモリを定義
  //  printf("arc_array:%p\n",arc_array);
    adr_node_arc = (_INFO_NODE_ARC*)malloc(sizeof(_INFO_NODE_ARC));//nodeとarcの情報のメモリを定義
  //nodeとarcの情報を初期化する

    adr_node_arc->node = node_array; //格子状のノードを格納する配列の先頭要素のアドレスを書き込み
    adr_node_arc->arc = arc_array;//格子状のアークを格納する配列の先頭要素のアドレスを書き込み
    adr_node_arc->num_node = n*n;//格子状のノードの個数を書き込み
    adr_node_arc->num_arc = 4*n*(n-1);//格子状のアークの個数を書き込み
    adr_node_arc->length = length;//アークの長さを書き込み

  //printf("gen_lattice - adr_node_arc:%p\n",adr_node_arc->node);

    //NODEの情報を初期化する   ---   OK

    for(int i = 0;i < (n*n); i++){//ノードのIDと座標と本ノードに接続されるアークの数を初期化する
        (node_array+i)->node_ID = i;
        (node_array+i)->x = 0;
        (node_array+i)->y = 0;
        (node_array+i)->connected_arc_num = 0;
        //本ノードに接続されるアークの集合を初期化にする(0にする)
           for(int j = 0 ; j < MAX_CONNECTED_ARC_NUM ; j++)
                (node_array+i)->connected_arc[j] = 0;
       for(int j = 0 ; j < MAX_CONNECTED_ARC_NUM ; j++)
                (node_array+i)->connected_node[j] = 0;

    }

    //NODEの座標値を書く   ---   OK
    for(int i = 0 ; i < n; i++) //下からノードの座標を書き込み
        for(int j = 0 ; j < n ; j++){ //左からノードの座標を書き込み
            (node_array+(j*n+i))->x = i * length ; //ノードのx座標を書き込み
            (node_array+(j*n+i))->y = j * length ; //ノードのy座標を書き込み
        }

    //ノードの情報を表示する   ---  OK
//    for(int i = 0;i < (n*n);i++)
//        printf("node%d:x-%f,y-%f,connected with %d nodes\n",(node_array+i)->node_ID,(node_array+i)->x,(node_array+i)->y,(node_array+i)->connected_arc_num);


    //ARCの情報を書く
        //ARCの情報を初期化    ---   OK
    for(int i = 0;i < 4*(n)*(n-1);i++){
        (arc_array+i)->arc_ID = i;
        (arc_array+i)->des_node = 0;
        (arc_array+i)->ori_node = 0;
    }
        //左からアークに接続されるノードを書き込み    ---   OK
        for(int j = 0; j < n ;j++)
            for(int k = 0 ; k < n-1;k++){
                (arc_array+(j*(n-1)+k))->ori_node = j*n+k; //アークの始発点を書き込み
            //    qDebug() <<(arc_array+(j*(n-1)+k))->arc_ID<<(arc_array+(j*(n-1)+k))->ori_node;
                (arc_array+(j*(n-1)+k))->des_node = j*n+k+1; //アークの終点を書き込み
            }

        //下からアークにに接続されるノードを書き込み    ---   OK
        for(int i = 0; i < n-1 ;i++)
            for(int j = 0;j < n ; j++){
                (arc_array+n*(n-1)+i*n+j)->ori_node = i*n+j;//アークの始発点を書き込み
                (arc_array+n*(n-1)+i*n+j)->des_node = (i+1)*n+j;//アークの終点を書き込み
            }
        //アークの長さを計算する
        for(int i = 0 ; i < 2*n*(n-1);i++){
            int start = (arc_array+i)->ori_node;
            int end = (arc_array+i)->des_node;
            (arc_array+i)->length=sqrt(((node_array+start)->x - (node_array+end)->x)*((node_array+start)->x - (node_array+end)->x) +
                    ((node_array+start)->y - (node_array+end)->y)*((node_array+start)->y - (node_array+end)->y));
        }

        //生成したアークのori_nodeとdes_nodeを逆します
        for(int i = 2*n*(n-1),j=0; i < 4*n*(n-1) ; i++,j++){
        (arc_array+i)->ori_node = (arc_array+j)->des_node;
        (arc_array+i)->des_node = (arc_array+j)->ori_node;
        (arc_array+i)->length   = (arc_array+j)->length;
        }





        //アークの情報を表示する    ---   OK
        for(int i = 0; i < 4*n*(n-1);i++)
            printf("Arc%d:%d-%d\n",(arc_array+i)->arc_ID,(arc_array+i)->ori_node,(arc_array+i)->des_node);
       //     printf("Arc %d  length = %f\n",(arc_array+i)->length);


        //ノードに接続されるノードの数を計算する   ---   OK
        for(int i = 0; i < 4 * n * (n-1) ; i++){ //ノードに１本のアークを介して直接的に接続されるノードを書き込み
        int start = 0 ;
        int end   = 0 ;
        start  = (arc_array+i)->ori_node;
        end    = (arc_array+i)->des_node;

        (node_array+start)->connected_arc[(node_array+start)->connected_arc_num]  =  i;
        (node_array+start)->connected_node[(node_array+start)->connected_arc_num] = end;
        (node_array+end)->connected_node[(node_array+end)->connected_arc_num] = start;
        (node_array+start)->connected_arc_num++;//１本アークの原点に直接的に接続されるノードの個数を増える
        (node_array+end)->connected_arc[(node_array+end)->connected_arc_num]  =  i;
        (node_array+end)->connected_arc_num++;//１本アークの終点に直接的に接続されるノードの個数を増える


        }

        //ノードに接続されるノードの数を表示する   ---   OK
        for(int i = 0 ; i < n * n;i++ )
            printf("node %d - %d\n",i,(node_array+i)->connected_arc_num);

    //ノードの情報を表示   ---   OK
//	for(int i = 0 ; i < n * n ; i++)
//		for(int j = 0 ; j < (node_array+i)->connected_arc_num ; j++){
//		printf("node-arc %d [%d] = %d\n",i,j,(node_array+i)->connected_arc[j]);
//		printf("node %d [%d] = %d\n",i,j,(node_array+i)->connected_node[j]);
//  }


        return adr_node_arc;




}

_INFO_NODE_ARC *gen_radial(int n, int m, int length) //放射環状計算関数
{
    node_array = (NODE*)malloc((n*m)*sizeof(NODE));//ノードのメモリを定義
    arc_array  = (ARC*)malloc(((4*n-1)*m)*sizeof(ARC));//アークのメモリを定義
    adr_node_arc = (_INFO_NODE_ARC*)malloc(sizeof(_INFO_NODE_ARC));//nodeとarcの情報のメモリを定義

    //nodeとarcの情報の情報を初期化する
    adr_node_arc->node = node_array;//放射環状ノードを格納する配列の先頭要素のアドレスを書き込み
    adr_node_arc->arc =arc_array;//放射環状アークを格納する配列の先頭要素のアドレスを書き込み
    adr_node_arc->num_node = n*m;//放射環状のノードの個数を書き込み
    adr_node_arc->num_arc = 2*(2*n-1)*m;//放射環状のアークの個数を書き込み
    adr_node_arc->length = length;//アークの長さを書き込み
    //nodeの情報を初期化
    for(int i = 0; i < n*m ;i++){
        (node_array+i)->node_ID = i;
        (node_array+i)->x = 0;
        (node_array+i)->y = 0;
        (node_array+i)->connected_arc_num = 0;
        //本ノードに接続されるアークの集合を初期化にする(0にする)
        for(int j = 0; j < MAX_CONNECTED_ARC_NUM;j++)
            (node_array+i)->connected_arc[j] = 0;
    }

    //NODEの座標値を書く
    for(int i = 0; i < n ;i++) //放射環状内部から外側に引く
        for(int j = 0 ;j < m ;j++){//放射環状時計回り方向にノードの座標を書き込み
            (node_array+i*m+j)->x = (i+1) * length * sin(j * (2*PI)/m);//x座標
            (node_array+i*m+j)->y = (i+1) * length * cos(j * (2*PI)/m); //y座標
        }

    //ARCの情報を書く
      // ARCを初期化
    for(int i = 0 ; i < 2*(2*n-1)*m ;i++){
        (arc_array+i)->arc_ID = i;
        (arc_array+i)->ori_node = 0;
        (arc_array+i)->des_node = 0;
        (arc_array+i)->length = 0;
    }


      //放射状のアークに接続されてるノードを書き込み
    for(int i = 0; i < n ; i++)//内部から外側に引くアークに接続されてるノードを書き込み
        for(int j = 0 ; j < m ; j++){//時計回り方向に接続されてるノードを書き込み
            (arc_array+i*m+j)->ori_node = i*m+j;
            if(j == m-1) //中心部になったら外側に引く
            (arc_array+i*m+j)->des_node = i*m;
            else  //中心部になってなかったら時計回り方向に
            (arc_array+i*m+j)->des_node = i*m+j+1;

        }



      //環状のアークに接続されてるノードを書き込み
    for(int i = 0 ; i < (n-1);i++)
        for(int j = 0 ; j < m ;j++){
        (arc_array+n*m+i*m+j)->ori_node =  i*m + j;//内部は原点にする
        (arc_array+n*m+i*m+j)->des_node = (i+1)*m + j;//外側は終点にする
        }

    //生成したアークのori_nodeとdes_nodeを逆します
    for(int i = (2*n-1)*m,j=0; i < 2*(2*n-1)*m ; i++,j++){
    (arc_array+i)->ori_node = (arc_array+j)->des_node;
    (arc_array+i)->des_node = (arc_array+j)->ori_node;
    }

    //アークの長さを計算する
   for(int i = 0 ; i < 2*(2*n-1)*m ;i++){
       int start = (arc_array+i)->ori_node;
       int end = (arc_array+i)->des_node;
       (arc_array+i)->length=sqrt(((node_array+start)->x - (node_array+end)->x)*((node_array+start)->x - (node_array+end)->x) +
               ((node_array+start)->y - (node_array+end)->y)*((node_array+start)->y - (node_array+end)->y));
   }

    //test
    for(int i = 0 ; i < 2*(2*n-1)*m ; i++)
        printf("ARC%d:%d-%d\n",(arc_array+i)->arc_ID,(arc_array+i)->ori_node,(arc_array+i)->des_node);
    //    printf("Arc %d length = %f \n",i,(arc_array+i)->length);


     //ノードに接続されるノードの数を計算する   ---   OK
        for(int i = 0; i < 2*(2*n-1)*m ; i++){ //ノードに１本のアークを介して直接的に接続されるノードを書き込み
        int start = 0 ;
        int end   = 0 ;
        start  = (arc_array+i)->ori_node;
        end    = (arc_array+i)->des_node;

        (node_array+start)->connected_arc[(node_array+start)->connected_arc_num]  =  i;
        (node_array+start)->connected_node[(node_array+start)->connected_arc_num] = end;
        (node_array+end)->connected_node[(node_array+end)->connected_arc_num] = start;
        (node_array+start)->connected_arc_num++;//１本アークの原点に直接的に接続されるノードの個数を増える
        (node_array+end)->connected_arc[(node_array+end)->connected_arc_num]  =  i;
        (node_array+end)->connected_arc_num++;//１本アークの終点に直接的に接続されるノードの個数を増える


        }

    //ノードに接続されるノードの数を表示する
//    for(int i = 0 ; i < n * m;i++ )
//        printf("node %d - %d\n",i,(node_array+i)->connected_arc_num);

    return adr_node_arc;

}

_INFO_NODE_ARC *gen_general_graph(char *filename)//東京データ計算関数
{
    adr_node_arc = (_INFO_NODE_ARC*)malloc(sizeof(_INFO_NODE_ARC));//nodeとarcの情報のメモリを定義
    FILE *fp_map;//ファイル対象を定義
    char map_file[100];//filenameを定義
    sprintf(map_file, "%s",filename);

    if((fp_map = fopen(map_file,"r"))==NULL){//データファイル読めない処理
        fprintf(stderr, "can't open %s . \n",map_file);
     }

    printf("\n Now reading file \"%s\"...\n\n",map_file);

    char data_type[100];
    char *str_node      = "node";
    char *str_num_node      = "num_node";
    char *str_arc       = "arc";
    char *str_num_arc       = "num_arc";
    int num_node = 0;
    int num_arc = 0;
    int loop_node = 0;//ノードのLOOP計算用
    int loop_arc = 0;//アークのLOOP計算用
    int int2double = 0;//int convert to double

    while(fscanf(fp_map,"%s",&data_type)!=EOF){//東京データ読み取り処理
        //ノードの個数を読み込む
        if(strncmp(data_type,str_num_node,strlen(str_num_node)) == COINCIDENCE){
            fscanf(fp_map,"%d",&(num_node));
            adr_node_arc->num_node = num_node;//ノードの個数を読んだらメモリを定義
            node_array = (NODE*)malloc((num_node)*sizeof(NODE));//ノードのメモリを定義
         }

        //アークの個数を読み込む
        if(strncmp(data_type,str_num_arc,strlen(str_num_arc)) == COINCIDENCE){
            fscanf(fp_map,"%d",&num_arc);
            adr_node_arc->num_arc = 2*num_arc;//アークの個数を読んだらメモリを定義
            arc_array = (ARC*)malloc(2*(num_arc)*sizeof(ARC));//ノードのメモリを定義
         }
        //ノードの情報を読み込む
        if(strncmp(data_type,str_node,strlen(str_node)) == COINCIDENCE){
            fscanf(fp_map,"%d",&(node_array+loop_node)->node_ID);
            fscanf(fp_map,"%d",&int2double);
            (node_array+loop_node)->x = (double)int2double;
            fscanf(fp_map,"%d",&int2double);
            (node_array+loop_node)->y = -(double)int2double;
            printf("node%d  x:%f y:%f\n",(node_array+loop_node)->node_ID,(node_array+loop_node)->x,(node_array+loop_node)->y);
            loop_node++;
        }
        //アークの情報を読み込む
        if(strncmp(data_type,str_arc,strlen(str_arc)) == COINCIDENCE){
            fscanf(fp_map,"%d",&(arc_array+loop_arc)->arc_ID);
            printf("arc_ID:%d  ",(arc_array+loop_arc)->arc_ID);
            fscanf(fp_map,"%d",&(arc_array+loop_arc)->ori_node);
            printf("ori:%d  ",(arc_array+loop_arc)->ori_node);
            fscanf(fp_map,"%d",&(arc_array+loop_arc)->des_node);
            printf("des:%d\n",(arc_array+loop_arc)->des_node);
            loop_arc++;
        }


   }
    fclose(fp_map);
    //ノードを初期化
    for(int i = 0 ; i < num_node ;i++){
        (node_array+i)->connected_arc_num = 0;
        for(int j = 0 ; j < MAX_CONNECTED_ARC_NUM ; j++){
            (node_array+i)->connected_arc[j] = 0;
            (node_array+i)->connected_node[j]= 0 ;
        }
    }



    //アークの長さを計算する
    for(int i = 0 ; i < num_arc ;i++){
        int start = (arc_array+i)->ori_node;
        int end = (arc_array+i)->des_node;
        (arc_array+i)->length=sqrt(((node_array+start)->x - (node_array+end)->x)*((node_array+start)->x - (node_array+end)->x) +
                ((node_array+start)->y - (node_array+end)->y)*((node_array+start)->y - (node_array+end)->y));
        printf("Arc %d length = %f\n",i,(arc_array+i)->length);
    }

    //生成したアークのori_nodeとdes_nodeを逆します
    for(int i = num_arc,j=0; i < 2*num_arc ; i++,j++){
    (arc_array+i)->ori_node = (arc_array+j)->des_node;
    (arc_array+i)->des_node = (arc_array+j)->ori_node;
      (arc_array+i)->length =  (arc_array+j)->length;
    }

    for(int i = 0; i < 2*num_arc ; i++){ //ノードに１本のアークを介して直接的に接続されるノードを書き込み
    int start = 0 ;
    int end   = 0 ;
    start  = (arc_array+i)->ori_node;
    end    = (arc_array+i)->des_node;

    (node_array+start)->connected_arc[(node_array+start)->connected_arc_num]  =  i;
    (node_array+start)->connected_node[(node_array+start)->connected_arc_num] = end;
    (node_array+end)->connected_node[(node_array+end)->connected_arc_num] = start;
    (node_array+start)->connected_arc_num++;//１本アークの原点に直接的に接続されるノードの個数を増える
    (node_array+end)->connected_arc[(node_array+end)->connected_arc_num]  =  i;
    (node_array+end)->connected_arc_num++;//１本アークの終点に直接的に接続されるノードの個数を増える


    }

    //ノードの情報
//    for(int i = 0 ; i < num_node ; i++){
//      printf("G-NODE %d was connected %d node,they are ",i,(node_array+i)->connected_arc_num);
//      for(int j = 0 ; j < (node_array+i)->connected_arc_num ; j++)
//          printf("%d ",(node_array+i)->connected_arc[j]);
//      printf("\n");
//    }

    //アークの情報
//    for(int i = 0 ; i < 2* num_arc ; i++){
//        printf("Arc %d  : %d-%d\n",i,(arc_array+i)->ori_node,(arc_array+i)->des_node);
//    }




    adr_node_arc->node = node_array;//東京データのノードを格納する配列の先頭要素のアドレスを転送
    adr_node_arc->arc = arc_array;//東京データのアークを格納する配列の先頭要素のアドレスを転送

    return adr_node_arc;
}

