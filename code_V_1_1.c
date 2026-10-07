#include<stdio.h>
//Suggestion Required

//It reads the sensor data
void sensor_data(int sensor[]){
    for(int i=0 ; i<8; i++){
        printf(" %d" , sensor[i]);
        
            
    }
    printf("\n");
    
}
// This gives the relative position of bot with white line
float bot_position(int sensor[]){
    float sum =0;
    float count = 0;
    for(int i =0; i<8; i++){
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

//it controls the motor rpm
int callback_actual_position(float pos){
     float error =pos;
      int left_motor , right_motor;
     if(error<0){
        left_motor = 120;
        printf("Bot is towards right\n");
        if(error<-1 || error == -1){
            right_motor = 120;
            right_motor = -(right_motor/error);
            printf("Left motor Speed :%d \n" , left_motor);
            return right_motor;
        }
        if(error>-1){
            right_motor = 195;
            left_motor = 200;
            printf("Left motor Speed :%d \n" , left_motor);
            return right_motor;
        }

     }
     if(error >0){
        right_motor = 120;
        printf("Bot is towards left \n");
        if(error ==999.00){
            printf("Robot Lost");
        }
        if(error>1 && error !=999){
        left_motor = 120;
        left_motor = (left_motor/error); 
        printf("Right motor Speed :%d \n" , right_motor);      
        return left_motor;
        }
        if(error<1 || error ==1){
            left_motor =195;
            right_motor = 200;
             printf("Right motor Speed :%d \n" , right_motor);
            return left_motor;
        }
     }
     if(error == 0){
        printf("Perfectly Aligned");
     }
 }
//Expected input by the sensor type of feedback
void left_move(int sensor[]){
    int new_sensor[8] = {0 , 0, 0, 0, 0, 0, 0, 0};
    for(int i =0 ; i<6; i++){
        if(sensor[i+1]==sensor[i] && sensor[i] ==1){
        sensor[i]=0;
        sensor[i+2]=1;
        printf("Sensor Value:");
        for(int j = 0 ; j<8; j++){
            new_sensor[j] = sensor[j];
            printf("%d" ,sensor[j]);
        }
        }
      printf("\n");
         float position = bot_position(new_sensor);
   if(position <0){
    int right_motor = callback_actual_position(position);
    printf("Right motor speed: %d\n" ,right_motor);
}
    if(position ==0){
        int  left_motor = 200;
        int right_motor = 200;
        printf("Motor speed should be: %d %d" , left_motor, right_motor);

    }
    if(position >0 && position !=999){
        int left_motor = {callback_actual_position(position)};
        printf("Left motor speed should be: %d \n" , left_motor);
    }
       if(sensor[3]==sensor[4] &&sensor[4] == 1){
            break;
        } 
        if(sensor[3]==sensor[4] &&sensor[4] == 1){
            break;
        }
        
         
    }
}

void right_move(int sensor[]){
    int new_sensor[8] = {0 , 0, 0, 0, 0, 0, 0, 0};
    for(int i =7; 0<i; i--){
        if(sensor[i]==sensor[i-1] && sensor[i] == 1){
            sensor[i] =0;
            sensor[i-2] =1;
             printf("Sensor Value: ");
        for(int j=0 ; j<8; j++){
            new_sensor[j] = sensor[j];
            printf("%d" , sensor[j]);
        }
        }
        printf("\n");
         float position = bot_position(new_sensor);
   if(position <0){
    int right_motor = callback_actual_position(position);
    printf("Right motor speed: %d\n" ,right_motor);
}
    if(position ==0){
        int  left_motor = 200;
        int right_motor = 200;
        printf("Motor speed should be: %d %d" , left_motor, right_motor);

    }
    if(position >0 && position !=0){
        int left_motor = {callback_actual_position(position)};
        printf("Left motor speed should be: %d \n" , left_motor);
    }
       if(sensor[3]==sensor[4] &&sensor[4] == 1){
            break;
        } 
    }
}
int main(){
   
   int req_position[8] = {0,0,0,1,1,0,0,0};
   int sensor[8] = {0, 0, 0, 0, 0 , 0 , 0 , 0};
   printf("Sensor Output: ");
   sensor_data(sensor);
   printf("%0.2f \n" , bot_position(sensor));
   float position = bot_position(sensor);
 //  for(int i=0 ;position<4 && position>-4 ; i++){
   if(position <0){
    int right_motor = callback_actual_position(position);
    printf("Right motor speed: %d\n" ,right_motor);
    left_move(sensor);
}
    if(position ==0){
        int  left_motor = 200;
        int right_motor = 200;
        printf("Motor speed: %d %d" , left_motor, right_motor);

    }
    if(position >0){
        int left_motor = {callback_actual_position(position)};
        printf("Left motor speed: %d \n" , left_motor);
        right_move(sensor);
    }
  // }
    return 0; 
}
