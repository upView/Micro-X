

class PID
{
public:

  
  PID(float* command, float* angle, float* rate, float* time, float P, float I, float D); //Constructor

  void compute(); //Update pid output

  float getP();						 
  float getI();						
  float getD();	
  
  void setP(float P);
  void setI(float I);
  void setD(float D);					  				
  
  void reset_I();
  float value();
  void hold();
  
private:

  float _P; // (P)roportional Tuning Parameter
  float _I; // (I)ntegral Tuning Parameter
  float _D; // (D)erivative Tuning Parameter

  float *_command; // Pointers to the angle demand
  float *_angle; // Pointers to the current angle      
  float *_time;  // Pointers to the loop time  
  float *_rate; // Pointers to the angular speed
  
  float _ITerm; //Integral term
  float _pid;  //pid output value

};


