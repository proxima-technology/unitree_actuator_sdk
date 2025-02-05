#include "serialPort/SerialPort.h"
#include "shm/shm.hpp"
#include "config.h"
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>

#define PRINT_DURRATION_STATS 1
#if PRINT_DURRATION_STATS
#define DURRATION_DATA_SIZE 1000
#endif

#define DEBUG_WRITE_RAWDATA 0

static volatile int keepRunning = 1;
void signal_handler(int signum) {
  std::cerr<<std::endl<<"catch signum = "<<signum<<std::endl;
    keepRunning = 0;
}

std::string getEnvVarOrExit(const char* varName) {
    char const* tmp = std::getenv(varName);
    if (tmp == nullptr) {
        std::cerr << "ERROR: " << varName << " が設定されていません" << std::endl;
        std::exit(1);
    }
    return std::string(tmp);
}

std::vector<double> get_offset(void) {

    std::vector<double> memorable_offset_pos;

    std::string SMALL_SKEPTRON_ID = getEnvVarOrExit("SMALL_SKEPTRON_ID");
    std::string SMALL_SKEPTRON_PATH = getEnvVarOrExit("SMALL_SKEPTRON_PATH");

    std::cerr << "SMALL_SKEPTRON_ID: "<< SMALL_SKEPTRON_ID << std::endl;
    std::cerr << "SMALL_SKEPTRON_PATH: "<< SMALL_SKEPTRON_PATH << std::endl;
    std::string OFFSET_FILE = SMALL_SKEPTRON_PATH + "/hardware/unitree_actuator_sdk/example/offset.txt";
    std::ifstream file(OFFSET_FILE);

    // TODO: 現在はSMALL_NIMBUSが1号機と2号機しか存在しないことを仮定してエラー処理をしているが、これをもう少し一般的にチェックできるようにする。
    // https://github.com/proxima-technology/small_nimbus_ws/issues/111
    if( std::stoi(SMALL_SKEPTRON_ID)!=1 && std::stoi(SMALL_SKEPTRON_ID)!=2 )
    {
      std::cerr << "ERROR: SMALL_SKEPTRON_ID should be 1 or 2" << std::endl;
      std::exit(1);
    }
    std::string line;
    std::string column0, column1, column2;
    if (file.is_open()) {
      while (getline(file, line)) {
        std::stringstream ss(line);
        ss >> column0 >> column1 >> column2;
        if (column0==SMALL_SKEPTRON_ID){
          memorable_offset_pos.push_back(std::stod(column1));
          memorable_offset_pos.push_back(std::stod(column2));
          break;
        }
      }
    file.close();
    std::cerr << "関節原点オフセットファイル\n"<< OFFSET_FILE << "\nを開くことができました。\n" << std::endl;
    }
    else {
      std::cerr << "ERROR: 関節原点オフセットファイル\n"<< OFFSET_FILE << "\nを開けませんでした。\n" << std::endl;
      std::exit(1);
    }

    return memorable_offset_pos;
}

const char* legmotor_device[NUM_LEGMOTOR]= { "/dev/ttyUSB0", "/dev/ttyUSB1" };
const int legmotor_id[NUM_LEGMOTOR] = {0,1};
std::vector<double> memorable_offset_pos = get_offset();
std::vector<double> legmotor_sensor_shared(num_data_legmotor_sensor, 0.0);
std::vector<double> legmotor_command_shared(num_data_legmotor_command, 0.0);

pthread_mutex_t legmotor_localmutex;

void motor_thread(int index)
{
  int is_host = 1;
  ProcComm *proc_comm_sensor;
  ProcComm *proc_comm_command;
  if(0==index)
  {
    proc_comm_sensor = new ProcComm(filename_data_legmotor_sensor, id_data_legmotor_sensor, num_data_legmotor_sensor, is_host);
    proc_comm_command = new ProcComm(filename_data_legmotor_command, id_data_legmotor_command, num_data_legmotor_command, is_host);
  }
  
  MotorCmd    cmd;
  MotorData   data;
  data.q = - memorable_offset_pos[index]*6.33;

  #if DEBUG_WRITE_RAWDATA
  int write_count = 0;
  std::string SMALL_SKEPTRON_PATH = getEnvVarOrExit("SMALL_SKEPTRON_PATH");
  std::string DEBUG_GOMOTOR_RAWDATA_FILE = SMALL_SKEPTRON_PATH + "/hardware/unitree_actuator_sdk/debugdata/gomotor_rawdata.csv";
  std::ofstream output_file;
  if(index==0)
  {
    output_file.open(DEBUG_GOMOTOR_RAWDATA_FILE,std::ios::out);
  }
  #endif

  int actuator_status = -1;
  if(ENABLE_LEGMOTOR > 0)
  {
    actuator_status = READY_FOR_ACTUATION_STATUS_IDX;
  }
  else
  {
    actuator_status = DISABLED_BY_CONFIG_STATUS_IDX;
  }

  #if (ENABLE_LEGMOTOR > 0)
  SerialPort serial(legmotor_device[index]);
  cmd.motorType = MotorType::GO_M8010_6;
  data.motorType = MotorType::GO_M8010_6;
  cmd.id    = legmotor_id[index];
  cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
  cmd.tau     = 0.0;
  cmd.kp   = 0.0;
  cmd.kd   = 0.0;
  serial.sendRecv(&cmd,&data);
  #endif

  pthread_mutex_lock(&legmotor_localmutex);
  int runtime_offset_rotation_count = 0;
  double runtime_offset_pos = 0.0;
  
  while(1)
  {
    runtime_offset_pos = memorable_offset_pos[index] + runtime_offset_rotation_count*2*M_PI/6.33;
    legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + index] = data.q/6.33 + runtime_offset_pos;
    if(std::abs(legmotor_sensor_shared[index])<(-1e-6 + M_PI/6.33))
    {
      break;
    }
    if(legmotor_sensor_shared[index]<-(M_PI/6.33))
    {
      runtime_offset_rotation_count++;
    }
    if(legmotor_sensor_shared[index]>(M_PI/6.33))
    {
      runtime_offset_rotation_count--;
    }
  }
  legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + index] = data.dq/6.33;
  legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + index] = data.tau*6.33;
  legmotor_sensor_shared[TEMPERATURE_OBS_IDX*NUM_LEGMOTOR + index] = data.temp;
  legmotor_sensor_shared[STATUS_OBS_IDX*NUM_LEGMOTOR + index] = actuator_status;
  // reverse just after read
  if(1==index)
  {
    legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + index] = - legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + index];
    legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + index] = - legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + index];
    legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + index] = - legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + index];
  }

  if(0==index)
  {
    proc_comm_sensor->write_stdvec(legmotor_sensor_shared);
  }
  pthread_mutex_unlock(&legmotor_localmutex);
  
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  double start_clock = ts.tv_sec + 0.000000001*ts.tv_nsec;
  #if PRINT_DURRATION_STATS
  int durration_data_index = 0;
  Eigen::VectorXd durration_data(DURRATION_DATA_SIZE);
  double previous_clock = 1.0 * start_clock;
  #endif


  std::cout<<std::endl<<"start thread main loop: motor index "<<index<<" runtime_offset_rotation_count:"<<runtime_offset_rotation_count<<std::endl<<std::endl;
  while(keepRunning) {
    
    pthread_mutex_lock(&legmotor_localmutex);
    double measured_pos = legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + index];
    double measured_vel = legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + index];
    double measured_trq = legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + index];
    double measured_temp = legmotor_sensor_shared[TEMPERATURE_OBS_IDX*NUM_LEGMOTOR + index];    

    if(0==index)
    {
      legmotor_command_shared = proc_comm_command->read_stdvec();
    }
    double torque_control = legmotor_command_shared[TORQUE_CMD_IDX*NUM_LEGMOTOR + index];
    double target_pos = legmotor_command_shared[POSITION_TARGET_IDX*NUM_LEGMOTOR + index];
    double target_vel = legmotor_command_shared[VELOCITY_TARGET_IDX*NUM_LEGMOTOR + index];
    double kp = legmotor_command_shared[P_GAIN_IDX*NUM_LEGMOTOR + index];
    double kd = legmotor_command_shared[D_GAIN_IDX*NUM_LEGMOTOR + index];
    pthread_mutex_unlock(&legmotor_localmutex);

    double torque_max = 23.5;
    torque_control = std::max(-torque_max, std::min(torque_max, torque_control));

    // reverse just before send
    double torque_control_for_print = torque_control;
    double target_pos_for_print = target_pos;
    double target_vel_for_print = target_vel;
    if(1==index)
    {
      torque_control = - torque_control;
      target_pos = - target_pos;
      target_vel = - target_vel;
    }
    
    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;
    cmd.id    = legmotor_id[index];
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
    cmd.tau = 0.0;
    cmd.kp   = 0.0;
    cmd.kd   = 0.0;
    #if (ENABLE_LEGMOTOR == 1)
    // torque control
    cmd.tau     = (float) (torque_control/6.33);
    // position control
    cmd.q   = (float) ( target_pos - runtime_offset_pos ) * 6.33;
    cmd.dq     = (float) target_vel * 6.33;
    cmd.kp   = (float) kp / ( 6.33 * 6.33 );
    cmd.kd   = (float) kd / ( 6.33 * 6.33 );
    #endif

    
    #if (ENABLE_LEGMOTOR > 0)
    serial.sendRecv(&cmd,&data);
    #endif

    //std::cout <<  "data.correct: " << data.correct <<  std::endl; // no information for emergency stop
    //std::cout <<  "motor.MError: " << data.MError <<  std::endl;

    
    pthread_mutex_lock(&legmotor_localmutex);
    legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + index] = data.q/6.33 + runtime_offset_pos;
    legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + index] = data.dq/6.33;
    legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + index] = data.tau*6.33;
    legmotor_sensor_shared[TEMPERATURE_OBS_IDX*NUM_LEGMOTOR + index] = data.temp;
    legmotor_sensor_shared[STATUS_OBS_IDX*NUM_LEGMOTOR + index] = actuator_status;
    // reverse just after read
    if(1==index)
    {
      legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + index] = - legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + index];
      legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + index] = - legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + index];
      legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + index] = - legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + index];
    }

    if(0==index)
    {
      proc_comm_sensor->write_stdvec(legmotor_sensor_shared);
    }
    pthread_mutex_unlock(&legmotor_localmutex);
    

    clock_gettime(CLOCK_MONOTONIC, &ts);
    double loop_clock = ts.tv_sec + 0.000000001*ts.tv_nsec;
    double elapsed_time = loop_clock - start_clock;

    #if (PRINT_DURRATION_STATS && (ENABLE_LEGMOTOR > 0))
    durration_data(durration_data_index) = loop_clock - previous_clock;
    previous_clock = loop_clock; 
    durration_data_index++;
    if(durration_data_index==DURRATION_DATA_SIZE)
    {
      double durration_mean = durration_data.mean();
      std::cout
        <<"["<<index<<"]"
        <<" sensor_pos: "<<measured_pos
        <<", vel: "<<measured_vel
        <<", trq:"<<measured_trq
        <<", temp: "<<measured_temp<<std::endl;
      std::cout
        <<"["<<index<<"]"
        <<" torque_control: "<<torque_control_for_print<<std::endl;
      std::cout
        <<"["<<index<<"]"
        <<" target_pos: "<<target_pos_for_print
        <<", target_vel: "<<target_vel_for_print
        <<", kp: "<<kp
        <<", kd: "<<kd<<std::endl;
      std::cout
        <<"["<<index<<"]"
        <<" durration mean: "<< durration_mean
        <<", stddev: "<< std::sqrt( ( durration_data.array() - durration_mean ).square().sum() / (DURRATION_DATA_SIZE-1))
        <<", worst: "<< durration_data.maxCoeff()<<"\n"<<std::endl;
      durration_data_index -= DURRATION_DATA_SIZE;
    }
    #endif
    

    // failsafe
    if( std::abs(measured_vel)>29.5 )
    {
      std::cout
        <<"["<<index<<"]"
        <<" Stop by failsafe system"<<std::endl;
      std::cout
        <<"["<<index<<"]"
        <<" sensor_pos: "<<measured_pos
        <<", vel: "<<measured_vel
        <<", trq:"<<measured_trq
        <<", temp: "<<measured_temp<<std::endl;
      break;
    }
  #if DEBUG_WRITE_RAWDATA
    if(write_count<10000)
    {
      if( std::abs(kp)>1e-5 || std::abs(kd)>1e-5 || std::abs(torque_control)>1e-5)
      {
	if(write_count==0){
         output_file
          << "elapsed_time,"
          << "data.q,"
          << "data.dq,"
          << "data.tau,"
          << "cmd.q,"
          << "cmd.dq,"
          << "cmd.tau,"
          << "kp,"
          << "kd" << std::endl;
	}
        output_file
          << elapsed_time <<","
          << data.q << ","
          << data.dq << ","
          << data.tau << ","
          << cmd.q << ","
          << cmd.dq << ","
          << cmd.tau << ","
          << cmd.kp << ","
          << cmd.kd << std::endl;
        write_count++;
      }
    }
  #endif

  }
  
  cmd.motorType = MotorType::GO_M8010_6;
  data.motorType = MotorType::GO_M8010_6;
  cmd.id    = legmotor_id[index];
  cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
  cmd.tau     = 0.0;
  cmd.kp   = 0.0;
  cmd.kd   = 0.0;
#if (ENABLE_LEGMOTOR > 0)
    serial.sendRecv(&cmd,&data);
#endif
    if(0==index)
    {
      delete proc_comm_sensor;
      delete proc_comm_command;
    }

  #if DEBUG_WRITE_RAWDATA
    if(index==0)
    {
      output_file.close();
    }
  #endif

}

void *legmotor0_thread(void *)
{
  motor_thread(0);
  std::cout<<std::endl<<"legmotor0_thread finish!"<<std::endl<<std::endl;
  return NULL;
}

void *legmotor1_thread(void *)
{
  motor_thread(1);
  std::cout<<std::endl<<"legmotor1_thread finish!"<<std::endl<<std::endl;
  return NULL;
}

void test_connection(int index)
{
  std::cout<<std::endl<<"connecting test: motor index ="<<index<<std::endl;  
  std::cout<<"legmotor_device[index] ="<<legmotor_device[index]<<std::endl;
  std::cout<<"legmotor_id[index] ="<<legmotor_id[index]<<std::endl;    
  MotorCmd    cmd;
  MotorData   data;
  SerialPort serial(legmotor_device[index]);
  cmd.motorType = MotorType::GO_M8010_6;
  data.motorType = MotorType::GO_M8010_6;
  cmd.id    = legmotor_id[index];
  cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
  cmd.tau     = 0.0;
  cmd.kp   = 0.0;
  cmd.kd   = 0.0;
  serial.sendRecv(&cmd,&data);
  std::cout<<"data.q/6.33 = "<<data.q/6.33<<std::endl;
}


int main(int argc, char *argv[])
{
  std::cout<<"ENABLE_LEGMOTOR: "<<ENABLE_LEGMOTOR<<std::endl;  
  legmotor_command_shared.resize(num_data_legmotor_command);
  for(int i=0;i<num_data_legmotor_command;i++)
  {
    legmotor_command_shared[i] = 0.0;
  }
  legmotor_sensor_shared.resize(num_data_legmotor_sensor);
  for(int i=0;i<num_data_legmotor_sensor;i++)
  {
    legmotor_sensor_shared[i] = 0.0;
  }
  
  signal(SIGINT, signal_handler); // killed by ctrl+C
  signal(SIGHUP, signal_handler); // killed by tmux kill-server


  // connecting test
  /*
  #if (ENABLE_LEGMOTOR > 0)
  for(int i=0;i<NUM_LEGMOTOR;i++)
  {
    test_connection(i);
    usleep(1000000);
  }
  std::cout<<std::endl<<"connecting test: all motor passed!"<<std::endl<<std::endl;  
  usleep(1000000);
  #endif
  */

  std::cout<<std::endl<<"main program start"<<std::endl<<std::endl;  
  pthread_mutex_init(&legmotor_localmutex, NULL);
  pthread_t legmotor0_tid, legmotor1_tid;
  pthread_create(&legmotor0_tid, NULL, legmotor0_thread, NULL);
  pthread_create(&legmotor1_tid, NULL, legmotor1_thread, NULL);
  pthread_join(legmotor0_tid,NULL);
  pthread_join(legmotor1_tid,NULL);


  std::cout<<std::endl<<"main program finish!"<<std::endl<<std::endl;
  return 0;
}
