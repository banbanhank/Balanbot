#ifndef NUMERICALTOOL_H
#define NUMERICALTOOL_H
#include <Arduino.h>

class Differentiator{
  private:
    float input_state[3];
    float output_state[3];
    uint32_t time;
  
  public:
    Differentiator(){
        initialStates();
        time = micros();
    }
    void initialStates(){
        for(int i=0; i<3; i++){
            input_state[i] = 0;
            output_state[i] = 0;
        } 
    }
    float differential(float input){
        /*
        input_state[2] = input_state[1];
        input_state[1] = input_state[0];
        input_state[0] = input;
        output_state[2] = output_state[1];
        output_state[1] = output_state[0];
        output_state[0] = 1.562 * output_state[1] 
                        - 0.6413 * output_state[2] 
                        + 7.839 * input_state[1] 
                        - 7.839 * input_state[2];
    
        return output_state[0];
        */
       
       double dt = (double)(micros() - time) / 1000000; // Calculate delta time
       time = micros();
       input_state[1] = input_state[0];
       input_state[0] = input;
       return (input_state[0] - input_state[1])/dt;
       
    }
};

class Integrator{
    private:
        uint32_t time;
        float input_[2];
        float output_[2];

    public:
        Integrator(){
            time = micros();
        }
        float integral(float input){
            double dt = (double)(micros() - time) / 1000000; // Calculate delta time
            time = micros();
            /*
            output_[0] = dt*input_[1] + output_[1];
            
            input_[1] = input_[0];
            input_[0] = input;
            output_[1] = output_[0];

            return output_[0];
            */
           //Serial.println(dt);
           input_[0] += input*dt;
           return input_[0];
        }

        void clear(){
            input_[0] = 0;
        }
};

#endif