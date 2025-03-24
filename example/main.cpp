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
#define NUM_USB_PORT 2
#define NUM_MOTOR_PER_USB_PORT 3
// TODO: NUM_LEGMOTOR, NUM_USB_PORT, NUM_MOTOR_PER_USB_PORTの整合性チェックの実装

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

    std::string MIDDLE_SKEPTRON_ID = getEnvVarOrExit("MIDDLE_SKEPTRON_ID");
    std::string MIDDLE_SKEPTRON_PATH = getEnvVarOrExit("MIDDLE_SKEPTRON_PATH");

    std::cerr << "MIDDLE_SKEPTRON_ID: "<< MIDDLE_SKEPTRON_ID << std::endl;
    std::cerr << "MIDDLE_SKEPTRON_PATH: "<< MIDDLE_SKEPTRON_PATH << std::endl;
    std::string OFFSET_FILE = MIDDLE_SKEPTRON_PATH + "/hardware/offset.txt";
    std::ifstream file(OFFSET_FILE);

    // TODO: 現在はMIDDLE_SKEPTRONが1号機しか存在しないことを仮定してエラー処理をしているが、これをもう少し一般的にチェックできるようにする。
    // https://github.com/proxima-technology/small_nimbus_ws/issues/111
    if( std::stoi(MIDDLE_SKEPTRON_ID)!=1 )
    {
      std::cerr << "ERROR: MIDDLE_SKEPTRON_ID should be 1" << std::endl;
      std::exit(1);
    }
    std::string line;
    std::string column0, column1, column2, column3, column4, column5, column6;
    if (file.is_open()) {
      while (getline(file, line)) {
        std::stringstream ss(line);
        ss >> column0 >> column1 >> column2 >> column3 >> column4 >> column5 >> column6; // TODO: fix hard coding
        if (column0==MIDDLE_SKEPTRON_ID){
          memorable_offset_pos.push_back(std::stod(column1));
          memorable_offset_pos.push_back(std::stod(column2));
          memorable_offset_pos.push_back(std::stod(column3));
          memorable_offset_pos.push_back(std::stod(column4));
          memorable_offset_pos.push_back(std::stod(column5));
          memorable_offset_pos.push_back(std::stod(column6));
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

const char* legmotor_device[NUM_USB_PORT]= { "/dev/ttyUSB0", "/dev/ttyUSB1" };
const int legmotor_id[NUM_LEGMOTOR] = {0,1,2,3,4,5};
std::vector<double> memorable_offset_pos = get_offset();
std::vector<double> legmotor_sensor_shared(num_data_legmotor_sensor, 0.0);
std::vector<double> legmotor_command_shared(num_data_legmotor_command, 0.0);

pthread_mutex_t legmotor_localmutex;

void motor_thread(int usb_port_index)
{
  int is_host = 1;
  ProcComm *proc_comm_sensor;
  ProcComm *proc_comm_command;
  if(0==usb_port_index)
  {
    proc_comm_sensor = new ProcComm(filename_data_legmotor_sensor, id_data_legmotor_sensor, num_data_legmotor_sensor, is_host);
    proc_comm_command = new ProcComm(filename_data_legmotor_command, id_data_legmotor_command, num_data_legmotor_command, is_host);
  }

  MotorCmd cmd[NUM_MOTOR_PER_USB_PORT];
  MotorData data[NUM_MOTOR_PER_USB_PORT];

  for(int i=0; i<NUM_MOTOR_PER_USB_PORT; i++)
  {
    int motor_id = usb_port_index*NUM_MOTOR_PER_USB_PORT + i;
    data[i].q = - memorable_offset_pos[motor_id]*6.33;
  }

  #if DEBUG_WRITE_RAWDATA
  int write_count = 0;
  std::string MIDDLE_SKEPTRON_PATH = getEnvVarOrExit("MIDDLE_SKEPTRON_PATH");
  std::string DEBUG_GOMOTOR_RAWDATA_FILE = MIDDLE_SKEPTRON_PATH + "/hardware/unitree_actuator_sdk/debugdata/gomotor_rawdata.csv";
  std::ofstream output_file;
  if(usb_port_index==0)
  {
    output_file.open(DEBUG_GOMOTOR_RAWDATA_FILE,std::ios::out);
  }
  #endif

  int actuator_status[NUM_MOTOR_PER_USB_PORT];
  for(int i=0; i<NUM_MOTOR_PER_USB_PORT; i++)
  {
    if(ENABLE_LEGMOTOR > 0)
    {
      actuator_status[i] = READY_FOR_ACTUATION_STATUS_IDX;
    }
    else
    {
      actuator_status[i] = DISABLED_BY_CONFIG_STATUS_IDX;
    }
  }

  SerialPort serial(legmotor_device[usb_port_index]);
  for(int i=0; i<NUM_MOTOR_PER_USB_PORT; i++)
  {
    int motor_id = usb_port_index*NUM_MOTOR_PER_USB_PORT + i;
    #if (ENABLE_LEGMOTOR > 0)
    cmd[i].motorType = MotorType::GO_M8010_6;
    data[i].motorType = MotorType::GO_M8010_6;
    cmd[i].id  = legmotor_id[motor_id];
    cmd[i].mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
    cmd[i].tau = 0.0;
    cmd[i].kp = 0.0;
    cmd[i].kd = 0.0;
    serial.sendRecv(&(cmd[i]), &(data[i]));
    #endif
  }

  pthread_mutex_lock(&legmotor_localmutex);
  int runtime_offset_rotation_count[NUM_MOTOR_PER_USB_PORT] = {0};
  double runtime_offset_pos[NUM_MOTOR_PER_USB_PORT] = {0.0};

  for(int i=0; i<NUM_MOTOR_PER_USB_PORT; i++)
  {

    int motor_id = usb_port_index*NUM_MOTOR_PER_USB_PORT + i;
    while(1)
    {
      runtime_offset_pos[i] = memorable_offset_pos[motor_id] + runtime_offset_rotation_count[i] * 2 * M_PI/6.33;
      legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + motor_id] = data[i].q/6.33 + runtime_offset_pos[i];
      if(std::abs(legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + motor_id])<(-1e-6 + M_PI/6.33))
      {
        break;
      }
      if(legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + motor_id]<-(M_PI/6.33))
      {
        runtime_offset_rotation_count[i]++;
      }
      if(legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + motor_id]>(M_PI/6.33))
      {
        runtime_offset_rotation_count[i]--;
      }
    }
    legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + motor_id] = data[i].dq/6.33;
    legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + motor_id] = data[i].tau*6.33;
    legmotor_sensor_shared[TEMPERATURE_OBS_IDX*NUM_LEGMOTOR + motor_id] = data[i].temp;
    legmotor_sensor_shared[STATUS_OBS_IDX*NUM_LEGMOTOR + motor_id] = actuator_status[i];
    // reverse just after read
    if( 4==motor_id || 5==motor_id )
    {
      legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + motor_id] = - legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + motor_id];
      legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + motor_id] = - legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + motor_id];
      legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + motor_id] = - legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + motor_id];
    }
  }

  if(0==usb_port_index)
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


  std::cout << "\nstart thread main loop\n";
  for(int i=0; i<NUM_MOTOR_PER_USB_PORT; i++)
  {
    std::cout << " runtime_offset_rotation_count:[" << usb_port_index << "]["<< i <<"] "<<runtime_offset_rotation_count[i]<<std::endl;
  }

  while(keepRunning) {

    double measured_pos[NUM_MOTOR_PER_USB_PORT];
    double measured_vel[NUM_MOTOR_PER_USB_PORT];
    double measured_trq[NUM_MOTOR_PER_USB_PORT];
    double measured_temp[NUM_MOTOR_PER_USB_PORT];

    double torque_control[NUM_MOTOR_PER_USB_PORT];
    double target_pos[NUM_MOTOR_PER_USB_PORT];
    double target_vel[NUM_MOTOR_PER_USB_PORT];
    double kp[NUM_MOTOR_PER_USB_PORT];
    double kd[NUM_MOTOR_PER_USB_PORT];

    double torque_control_for_print[NUM_MOTOR_PER_USB_PORT];
    double target_pos_for_print[NUM_MOTOR_PER_USB_PORT];
    double target_vel_for_print[NUM_MOTOR_PER_USB_PORT];

    pthread_mutex_lock(&legmotor_localmutex);
    for(int i=0; i<NUM_MOTOR_PER_USB_PORT; i++)
    {
      int motor_id = usb_port_index*NUM_MOTOR_PER_USB_PORT + i;
      measured_pos[i] = legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + motor_id];
      measured_vel[i] = legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + motor_id];
      measured_trq[i] = legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + motor_id];
      measured_temp[i] = legmotor_sensor_shared[TEMPERATURE_OBS_IDX*NUM_LEGMOTOR + motor_id];
    }

    if(0==usb_port_index)
    {
      legmotor_command_shared = proc_comm_command->read_stdvec();
    }

    for(int i=0; i<NUM_MOTOR_PER_USB_PORT; i++)
    {
      int motor_id = usb_port_index*NUM_MOTOR_PER_USB_PORT + i;
      torque_control[i] = legmotor_command_shared[TORQUE_CMD_IDX*NUM_LEGMOTOR + motor_id];
      target_pos[i] = legmotor_command_shared[POSITION_TARGET_IDX*NUM_LEGMOTOR + motor_id];
      target_vel[i] = legmotor_command_shared[VELOCITY_TARGET_IDX*NUM_LEGMOTOR + motor_id];
      kp[i] = legmotor_command_shared[P_GAIN_IDX*NUM_LEGMOTOR + motor_id];
      kd[i] = legmotor_command_shared[D_GAIN_IDX*NUM_LEGMOTOR + motor_id];
    }
    pthread_mutex_unlock(&legmotor_localmutex);

    // set cmd
    double torque_max = 23.5;
    for(int i=0; i<NUM_MOTOR_PER_USB_PORT; i++)
    {
      int motor_id = usb_port_index*NUM_MOTOR_PER_USB_PORT + i;

      torque_control[i] = std::max(-torque_max, std::min(torque_max, torque_control[i]));

      // reverse just before send
      torque_control_for_print[i] = torque_control[i];
      target_pos_for_print[i] = target_pos[i];
      target_vel_for_print[i] = target_vel[i];
      if( 4==motor_id || 5==motor_id )
      {
        torque_control[i] = - torque_control[i];
        target_pos[i] = - target_pos[i];
        target_vel[i] = - target_vel[i];
      }
    
      cmd[i].motorType = MotorType::GO_M8010_6;
      data[i].motorType = MotorType::GO_M8010_6;
      cmd[i].id    = legmotor_id[motor_id];
      cmd[i].mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
      cmd[i].tau = 0.0;
      cmd[i].kp   = 0.0;
      cmd[i].kd   = 0.0;
      #if (ENABLE_LEGMOTOR == 1)
      // torque control
      cmd[i].tau     = (float) (torque_control[i]/6.33);
      // position control
      cmd[i].q   = (float) ( target_pos[i] - runtime_offset_pos[i] ) * 6.33;
      cmd[i].dq     = (float) target_vel[i] * 6.33;
      cmd[i].kp   = (float) kp[i] / ( 6.33 * 6.33 );
      cmd[i].kd   = (float) kd[i] / ( 6.33 * 6.33 );
      #endif

      #if (ENABLE_LEGMOTOR > 0)
      serial.sendRecv(&(cmd[i]), &(data[i]));
      #endif
    }

    pthread_mutex_lock(&legmotor_localmutex);
    for(int i=0; i<NUM_MOTOR_PER_USB_PORT; i++)
    {
      int motor_id = usb_port_index*NUM_MOTOR_PER_USB_PORT + i;
      legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + motor_id] = data[i].q/6.33 + runtime_offset_pos[i];
      legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + motor_id] = data[i].dq/6.33;
      legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + motor_id] = data[i].tau*6.33;
      legmotor_sensor_shared[TEMPERATURE_OBS_IDX*NUM_LEGMOTOR + motor_id] = data[i].temp;
      legmotor_sensor_shared[STATUS_OBS_IDX*NUM_LEGMOTOR + motor_id] = actuator_status[i];
      // reverse just after read
      if( 4==motor_id || 5==motor_id )
      {
        legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + motor_id] = - legmotor_sensor_shared[POSITION_OBS_IDX*NUM_LEGMOTOR + motor_id];
        legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + motor_id] = - legmotor_sensor_shared[VELOCITY_OBS_IDX*NUM_LEGMOTOR + motor_id];
        legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + motor_id] = - legmotor_sensor_shared[TORQUE_OBS_IDX*NUM_LEGMOTOR + motor_id];
      }
    }

    if( 0==usb_port_index )
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

      for(int i=0; i<NUM_MOTOR_PER_USB_PORT; i++)
      {
        int motor_id = usb_port_index*NUM_MOTOR_PER_USB_PORT + i;
        std::cout
          <<"["<<usb_port_index<<"]"<<"["<<i<<"]"
          <<" sensor_pos: "<<measured_pos[i]
          <<", vel: "<<measured_vel[i]
          <<", trq:"<<measured_trq[i]
          <<", temp: "<<measured_temp[i]<<std::endl;
          std::cout
          <<"["<<usb_port_index<<"]"<<"["<<i<<"]"
          <<" torque_control: "<<torque_control_for_print[i]<<std::endl;
        std::cout
          <<"["<<usb_port_index<<"]"<<"["<<i<<"]"
          <<" target_pos: "<<target_pos_for_print[i]
          <<", target_vel: "<<target_vel_for_print[i]
          <<", kp: "<<kp[i]
          <<", kd: "<<kd[i]<<std::endl;
      }
      std::cout
        <<"["<<usb_port_index<<"]"
        <<" durration mean: "<< durration_mean
        <<", stddev: "<< std::sqrt( ( durration_data.array() - durration_mean ).square().sum() / (DURRATION_DATA_SIZE-1))
        <<", worst: "<< durration_data.maxCoeff()<<"\n"<<std::endl;
      durration_data_index -= DURRATION_DATA_SIZE;
    }
    #endif
    

    // failsafe
    int fail_safe_flag = 0;
    for(int i=0; i<NUM_MOTOR_PER_USB_PORT; i++)
    {
      if( std::abs(measured_vel[i])>29.5 )
      {
        std::cout
          <<"["<<usb_port_index<<"]"
          <<" Stop by failsafe system"<<std::endl;
        std::cout
          <<"["<<usb_port_index<<"]"<<"["<<i<<"]"
          <<" sensor_pos: "<<measured_pos
          <<", vel: "<<measured_vel
          <<", trq:"<<measured_trq
          <<", temp: "<<measured_temp<<std::endl;
        fail_safe_flag = 1;
      }
    }
    if(fail_safe_flag)
    {
      break;
    }

  }

  for(int i=0; i<NUM_MOTOR_PER_USB_PORT; i++)
  {
    int motor_id = usb_port_index*NUM_MOTOR_PER_USB_PORT + i;
    cmd[i].motorType = MotorType::GO_M8010_6;
    data[i].motorType = MotorType::GO_M8010_6;
    cmd[i].id    = legmotor_id[motor_id];
    cmd[i].mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
    cmd[i].tau = 0.0;
    cmd[i].kp = 0.0;
    cmd[i].kd = 0.0;
#if (ENABLE_LEGMOTOR > 0)
    serial.sendRecv(&(cmd[i]), &(data[i]));
#endif
  }

  if(0==usb_port_index)
  {
    delete proc_comm_sensor;
    delete proc_comm_command;
  }

#if DEBUG_WRITE_RAWDATA
  if(usb_port_index==0)
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

// void test_connection(int usb_port_index)
// {
//   std::cout<<std::endl<<"connecting test: motor usb_port_index ="<<usb_port_index<<std::endl;
//   std::cout<<"legmotor_device[usb_port_index] ="<<legmotor_device[usb_port_index]<<std::endl;
//   std::cout<<"legmotor_id[usb_port_index] ="<<legmotor_id[usb_port_index]<<std::endl;
//   MotorCmd    cmd;
//   MotorData   data;
//   SerialPort serial(legmotor_device[usb_port_index]);
//   cmd.motorType = MotorType::GO_M8010_6;
//   data.motorType = MotorType::GO_M8010_6;
//   cmd.id    = legmotor_id[usb_port_index];
//   cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
//   cmd.tau     = 0.0;
//   cmd.kp   = 0.0;
//   cmd.kd   = 0.0;
//   serial.sendRecv(&cmd,&data);
//   std::cout<<"data.q/6.33 = "<<data.q/6.33<<std::endl;
// }


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

  std::cout<<std::endl<<"main program start\n"<<std::endl;
  pthread_mutex_init(&legmotor_localmutex, NULL);
  pthread_t legmotor0_tid, legmotor1_tid;
  pthread_create(&legmotor0_tid, NULL, legmotor0_thread, NULL);
  pthread_create(&legmotor1_tid, NULL, legmotor1_thread, NULL);
  pthread_join(legmotor0_tid,NULL);
  pthread_join(legmotor1_tid,NULL);
  std::cout<<std::endl<<"main program finish!\n"<<std::endl;
  return 0;
}
