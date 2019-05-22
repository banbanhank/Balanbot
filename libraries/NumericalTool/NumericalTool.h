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
        input_state[1] = input_state[0];
        input_state[0] = input;
        return (input_state[0]-input_state[1]);
        
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
           input_[0] += input;
           return input_[0];
           
        }
};

#endif