#include<stdio.h>

void sensor_data(int sensor[]){
    for(int i=0 ; i<8; i++){
        printf(" %d" , sensor[i]);
        
            
    }
    
}

float bot_position(int sensor[]){
    float sum =0;
    float count = 0;
    for(int i =0; i<8; i++){
        // if(sensor[i] ==sensor[i+1] && sensor[i] == 1){
        // float pos = i-3.5;
        // return pos;
        // }
        if(sensor[i] ==1){
            count +=1;
            sum = sum +(i-3.5);
        }
    }
    if(count ==0){
        return 999;
    }
    
    return sum/count;
}

int left_move(int sensor[]){
    for(int i =0 ; i<6; i++){
        if(sensor[i+1]==sensor[i] && sensor[i] ==1){
        sensor[i]=0;
        sensor[i+2]=1;
        }
        printf("Sensor Value:");
        for(int j = 0 ; j<8; j++){
            printf("%d " ,sensor[j]);
        }
        if(sensor[3]==sensor[4] &&sensor[4] == 1){
            break;
        }
         
    }
}

// int callback_actual_position(int sensor[] , int pos){
//     int error = 0-pos;
// }

int main(){
   int req_position[8] = {0,0,0,1,1,0,0,0};
   int sensor[8] = {0, 0, 0, 0, 0 , 0 , 0 , 0};
   printf("Sensor Output: ");
   sensor_data(sensor);
   printf("%0.2f \n" , bot_position(sensor));
   float position = bot_position(sensor);
 //  for(int i=0 ;position<4 && position>-4 ; i++){
   if(position <0){
    left_move(sensor);
}
    if(position ==0){
        printf("Aligned Successfully \n");
    }
  // }
    return 0; 
}