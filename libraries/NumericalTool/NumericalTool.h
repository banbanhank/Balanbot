#ifndef NUMERICALTOOL_H
#define NUMERICALTOOL_H

class Differentiator{
  private:
    float input_state[3];
    float output_state[3];
  
  public:
    Differentiator(){
        initialStates();
    }
    void initialStates(){
        for(int i=0; i<3; i++){
            input_state[i] = 0;
            output_state[i] = 0;
        } 
    }
    float differential(float input){
        
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
        /*
        input_state[1] = input_state[0];
        input_state[0] = input;
        return input_state[0]-input_state[1];
        */
    }
};

class Integrator{
    private:
        float ts_;
        float input_[2];
        float output_[2];

    public:
        Integrator(){ ts_ = 0.01; }
        void setTs(float ts){ ts_ = ts; }
        float integral(float input){
            
            output_[0] = ts_*input_[1] + output_[1];
            
            input_[1] = input_[0];
            input_[0] = input;
            output_[1] = output_[0];

            return output_[0];
            /*
           input_[0] += input;
           return input_[0];
           */
        }
};

#endif